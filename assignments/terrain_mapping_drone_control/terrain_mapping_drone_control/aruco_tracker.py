#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped

from cv_bridge import CvBridge
import cv2
import numpy as np
import math

from tf2_ros import TransformBroadcaster
from transforms3d.euler import mat2euler


class ArucoTracker(Node):
    def __init__(self):
        super().__init__('aruco_tracker')
        self.cv_bridge = CvBridge()
        self.frame_counter = 0

        self.marker_size = 0.8  # meters
        self.calibration_received = False

        # Default camera calibration
        self.camera_matrix = np.array([
            [554.254691191187, 0.0, 320.5],
            [0.0, 554.254691191187, 240.5],
            [0.0, 0.0, 1.0]
        ])
        self.dist_coeffs = np.zeros(5)

        self.init_aruco()
        self.init_publishers()
        self.init_subscribers()
        self.get_logger().info(f'Camera matrix: \n{self.camera_matrix}')

    def init_aruco(self):
        try:
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            self.get_logger().info('OpenCV 4.7+ ArUco detector initialized')
        except AttributeError:
            self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
            self.aruco_params = cv2.aruco.DetectorParameters_create()
            self.detector = None
            self.get_logger().info('Fallback to older OpenCV ArUco API')

    def init_publishers(self):
        self.debug_image_pub = self.create_publisher(Image, '/aruco/debug_image', 10)
        self.marker_pose_pub = self.create_publisher(String, '/aruco/marker_pose', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

    def init_subscribers(self):
        self.create_subscription(Image, '/drone/down_mono', self.image_callback, 10)
        qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(CameraInfo, '/drone/down_mono/camera_info', self.camera_info_callback, qos_profile=qos)

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.k).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d)
        self.calibration_received = True

    def image_callback(self, msg):
        if not self.calibration_received:
            self.get_logger().debug('Using default calibration')

        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'mono8')
            self.frame_counter += 1
            corners, ids, _ = self.detect_markers(cv_image)

            debug_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
            if ids is not None:
                self.process_markers(debug_image, corners, ids)

            debug_msg = self.cv_bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
            self.debug_image_pub.publish(debug_msg)

        except Exception as e:
            self.get_logger().error(f'Image callback error: {e}')

    def detect_markers(self, image):
        try:
            if self.detector:
                return self.detector.detectMarkers(image)
            return cv2.aruco.detectMarkers(image, self.aruco_dict, parameters=self.aruco_params)
        except Exception as e:
            self.get_logger().error(f'Marker detection error: {e}')
            return [], None, []

    def process_markers(self, image, corners, ids):
        for i in range(len(ids)):
            if self.detector:
                marker_points = np.array([
                    [-self.marker_size/2, self.marker_size/2, 0],
                    [self.marker_size/2, self.marker_size/2, 0],
                    [self.marker_size/2, -self.marker_size/2, 0],
                    [-self.marker_size/2, -self.marker_size/2, 0]
                ], dtype=np.float32)

                success, rvec, tvec = cv2.solvePnP(marker_points, corners[i], self.camera_matrix, self.dist_coeffs)

                if success:
                    self.publish_transform(ids[i][0], rvec, tvec)
                    self.draw_marker_info(image, corners[i], ids[i][0], tvec)

    def publish_transform(self, marker_id, rvec, tvec):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'camera_frame'
        transform.child_frame_id = f'aruco_marker_{marker_id}'

        rot_matrix, _ = cv2.Rodrigues(rvec)
        euler = mat2euler(rot_matrix)
        quat = self.euler_to_quaternion(*euler)

        transform.transform.translation.x = tvec[0][0]
        transform.transform.translation.y = tvec[1][0]
        transform.transform.translation.z = tvec[2][0]
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(transform)

        pose = String()
        pose.data = f"Marker {marker_id}: x={tvec[0][0]:.2f} y={tvec[1][0]:.2f} z={tvec[2][0]:.2f}"
        self.marker_pose_pub.publish(pose)

    def draw_marker_info(self, image, corner, marker_id, tvec):
        center = corner[0].mean(axis=0)
        self.draw_crosshair(image, center)
        text = f"id:{marker_id} x:{tvec[0][0]:.2f} y:{tvec[1][0]:.2f} z:{tvec[2][0]:.2f}"
        pos = (int(center[0]), int(center[1] - 20))
        cv2.putText(image, text, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    def draw_crosshair(self, image, center, size=20, color=(0, 0, 255), thickness=2):
        x, y = int(center[0]), int(center[1])
        cv2.line(image, (x - size, y), (x + size, y), color, thickness)
        cv2.line(image, (x, y - size), (x, y + size), color, thickness)
        cv2.circle(image, (x, y), 2, color, thickness)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return [qx, qy, qz, qw]


def main(args=None):
    rclpy.init(args=args)
    node = ArucoTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
