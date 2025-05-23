import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

checkerboard_dimensions = (9, 7)

def generate_world_points(checkerboard_dim=checkerboard_dimensions, square_size=0.0186):
    objp = np.zeros((checkerboard_dim[0]*checkerboard_dim[1],3), np.float32)
    objp[:,:2] = np.mgrid[0:checkerboard_dim[0],0:checkerboard_dim[1]].T.reshape(-1,2)
    objp *= square_size
    return objp

def generate_world_points_R(checkerboard_dim=checkerboard_dimensions, square_size=0.0186):
    objp = np.zeros((checkerboard_dim[0]*checkerboard_dim[1],3), np.float32)
    objp[:,:2] = np.mgrid[0:checkerboard_dim[0],0:checkerboard_dim[1]].T.reshape(-1,2)
    objp *= square_size

    ## transform points to {R} frame
    T_R_checkerboard = np.eye(4)
    T_R_checkerboard[0,0] = 1
    T_R_checkerboard[1,1] = -1
    T_R_checkerboard[2,2] = -1
    T_R_checkerboard[0,3] = 0.0423
    T_R_checkerboard[1,3] = 0.1016
    T_R_checkerboard[2,3] = 0

    objp_h = np.hstack((objp, np.ones((objp.shape[0], 1))))
    objp_t = T_R_checkerboard.dot(objp_h.T)
    objp = objp_t[:3, :]
    objp = objp.T

    return objp

def image2world(imgpoint, camera_intrinsics, T_camera_world, z=0):
    R = T_camera_world[:3,:3]
    t = T_camera_world[:3, 3]

    R_inv = np.linalg.inv(R)
    intrinsic_inv = np.linalg.inv(camera_intrinsics)

    P = camera_intrinsics.dot(R)

    inverse_P = np.linalg.inv(P)
    unrotate_t = R_inv.dot(t)

    imgpoint_h = np.array([imgpoint[0], imgpoint[1], 1]).reshape((3,1))

    ## s * [(inverse_P*imgpoint_h)[2]] = z + unrotate_t[2]    
    s = (z + unrotate_t[2]) / ((inverse_P.dot(imgpoint_h))[2])
    world_point = R_inv.dot(intrinsic_inv.dot(imgpoint_h.dot(s)) - t )

    return world_point


def draw(img, imgpts):
    imgpts = imgpts.astype("int32")
    origin = tuple(imgpts[0].ravel())
    img = cv2.line(img, origin, tuple(imgpts[3].ravel()), (255,0,0), 5)
    img = cv2.line(img, origin, tuple(imgpts[2].ravel()), (0,255,0), 5)
    img = cv2.line(img, origin, tuple(imgpts[1].ravel()), (0,0,255), 5)
    return img

class CamToWorld(Node):
    def __init__(self):
        super().__init__('cam_to_world')
        self.image_sub = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10)
        self.image_info_sub = self.create_subscription(CameraInfo, '/camera_info', self.camera_info_callback, 10)
        self.br = CvBridge()

        self.k_matrix = []
        self.distortion_params = []
        self.got_camera_info = False

    def camera_info_callback(self, data):
        print("Got camera info")
        self.k_matrix = np.reshape(np.array(data.k), (3, 3))
        self.distortion_params = np.array(data.d)
        self.got_camera_info = True
        self.destroy_subscription(self.image_info_sub)
        
    
    def image_callback(self, data):
        if self.got_camera_info:
            self.get_logger().info('Receiving video frame')
        else:
            self.get_logger().info('Waiting for camera info')
        current_frame = self.br.imgmsg_to_cv2(data)
        rgb_image = cv2.cvtColor(current_frame, cv2.COLOR_RGB2BGR) ## ROS is RGB, OpenCV is BGR

        ## detect checkerboard

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        gray = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)

        ret, corners = cv2.findChessboardCorners(gray, checkerboard_dimensions, None)
        if ret:
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            cv2.drawChessboardCorners(rgb_image, checkerboard_dimensions, corners2, ret)

            ## solve PnP
            world_points = generate_world_points_R()
            # world_points = generate_world_points()
            ret_pnp, rvecs, tvecs = cv2.solvePnP(world_points, corners2, self.k_matrix, self.distortion_params, flags=cv2.SOLVEPNP_ITERATIVE)

            if ret_pnp:
                axis = np.float32([[0, 0, 0], [0.05,0,0], [0,0.05,0], [0,0,0.05]]).reshape(-1,3)
                imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, self.k_matrix, self.distortion_params)
                rgb_image = draw(rgb_image,imgpts)
                T_cam_world = np.eye(4)
                T_cam_world[:3, :3] = cv2.Rodrigues(rvecs)[0]
                T_cam_world[:3, 3] = tvecs.T
                self.get_logger().info('Got matrix\n'+np.array2string(T_cam_world, precision=5, separator=',', suppress_small=True))

                ## find red circle
                captured_frame_hsv = cv2.cvtColor(current_frame, cv2.COLOR_RGB2Lab)
                captured_frame_hsv_red = cv2.inRange(captured_frame_hsv, np.array([70, 150, 100]), np.array([100, 255, 255]))
                captured_frame_hsv_red = cv2.GaussianBlur(captured_frame_hsv_red, (5, 5), 2, 2)
                circles = cv2.HoughCircles(captured_frame_hsv_red, cv2.HOUGH_GRADIENT, 1, captured_frame_hsv_red.shape[0] / 8, param1=100, param2=18, minRadius=5, maxRadius=60)
                if circles is not None:
                    for (x, y, r) in circles[0]:
                        imgpoint_circle = np.array([x, y])
                        self.get_logger().info('Circle location:\n' + np.array2string(image2world(imgpoint_circle, self.k_matrix, T_cam_world, z=0), suppress_small=True, precision=5))
                    circles = np.round(circles[0, :]).astype("int")
                    for (x, y, r) in circles:
                        cv2.circle(rgb_image, (x, y), r, (0, 255, 0), 2)

        cv2.imshow("Original", rgb_image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    cam2world = CamToWorld()
    rclpy.spin(cam2world)
    cam2world.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()