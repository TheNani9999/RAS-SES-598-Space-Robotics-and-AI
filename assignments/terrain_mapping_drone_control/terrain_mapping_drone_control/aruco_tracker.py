import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

import cv2
import cv2.aruco as aruco
import numpy as np
import math
import time


class ArucoTracker(Node):

    def __init__(self):
        super().__init__('aruco_tracker')

        # Parameters
        self.declare_parameter('marker_size', 0.05)  # in meters
        self.declare_parameter('aruco_dictionary_id', "DICT_4X4_50")

        self.marker_size = self.get_parameter('marker_size').get_parameter_value().double_value
        aruco_dict_id_str = self.get_parameter('aruco_dictionary_id').get_parameter_value().string_value

        self.aruco_dict = self.get_aruco_dict(aruco_dict_id_str)
        self.aruco_params = aruco.DetectorParameters_create()

        self.bridge = CvBridge()
        self.calibration_received = False
        self.camera_matrix = None
        self.dist_coeffs = None

        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribers
        self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.create_subscription(CameraInfo, '/camera_info', self.camera_info_callback, 10)

        # Publishers
        self.marker_pub = self.create_publisher(String, 'marker_pose', 10)
        self.debug_image_pub = self.create_publisher(Image, 'debug_image', 10)

        self.get_logger().info('Aruco Tracker Node Initialized.')

    def get_aruco_dict(self, dict_name):
        aruco_dicts = {
            "DICT_4X4_50": aruco.DICT_4X4_50,
            "DICT_4X4_100": aruco.DICT_4X4_100,
            "DICT_4X4_250": aruco.DICT_4X4_250,
            "DICT_4X4_1000": aruco.DICT_4X4_1000,
        }
        return aruco.Dictionary_get(aruco_dicts.get(dict_name, aruco.DICT_4X4_50))

    def camera_info_callback(self, msg):
        if not self.calibration_received:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)
            self.calibration_received = True
            self.get_logger().info("Camera calibration data received.")

    def image_callback(self, msg):
        if not self.calibration_received:
            self.get_logger().warning("Waiting for camera calibration...")
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        debug_image = frame.copy()

        # Detect ArUco markers
        corners, ids, _ = aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            aruco.drawDetectedMarkers(debug_image, corners, ids)

            for i in range(len(ids)):
                rvec, tvec, _ = cv2.solvePnP(
                    objectPoints=np.array([
                        [-self.marker_size / 2, self.marker_size / 2, 0],
                        [self.marker_size / 2, self.marker_size / 2, 0],
                        [self.marker_size / 2, -self.marker_size / 2, 0],
                        [-self.marker_size / 2, -self.marker_size / 2, 0]
                    ]),
                    imagePoints=corners[i],
                    cameraMatrix=self.camera_matrix,
                    distCoeffs=self.dist_coeffs
                )

                # Draw axis
                aruco.drawAxis(debug_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.03)

                # Convert rotation vector to Euler angles
                rot_mat, _ = cv2.Rodrigues(rvec)
                yaw = math.atan2(rot_mat[1, 0], rot_mat[0, 0])
                pitch = math.atan2(-rot_mat[2, 0], math.sqrt(rot_mat[2, 1]**2 + rot_mat[2, 2]**2))
                roll = math.atan2(rot_mat[2, 1], rot_mat[2, 2])

                # Publish transform
                t = TransformStamped()
                t.header.stamp = msg.header.stamp
                t.header.frame_id = "camera_link"
                t.child_frame_id = f"aruco_marker_{ids[i][0]}"
                t.transform.translation.x = tvec[0][0]
                t.transform.translation.y = tvec[1][0]
                t.transform.translation.z = tvec[2][0]
                q = self.euler_to_quaternion(roll, pitch, yaw)
                t.transform.rotation.x = q[0]
                t.transform.rotation.y = q[1]
                t.transform.rotation.z = q[2]
                t.transform.rotation.w = q[3]

                self.tf_broadcaster.sendTransform(t)

                # Publish pose info
                marker_str = f"ID: {ids[i][0]}, Pos: [{tvec[0][0]:.2f}, {tvec[1][0]:.2f}, {tvec[2][0]:.2f}], YPR: [{math.degrees(yaw):.1f}, {math.degrees(pitch):.1f}, {math.degrees(roll):.1f}]"
                self.marker_pub.publish(String(data=marker_str))
                self.get_logger().info(marker_str)

        # Publish debug image
        debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
        self.debug_image_pub.publish(debug_msg)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - \
             math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + \
             math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - \
             math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + \
             math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return [qx, qy, qz, qw]


def main(args=None):
    rclpy.init(args=args)
    node = ArucoTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
