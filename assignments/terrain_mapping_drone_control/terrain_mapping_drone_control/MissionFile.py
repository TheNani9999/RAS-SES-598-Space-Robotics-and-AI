import math
import time
import re
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.logging import LoggingSeverity

from sensor_msgs.msg import Image, CameraInfo
from px4_msgs.msg import (
    VehicleOdometry,
    OffboardControlMode,
    VehicleCommand,
    TrajectorySetpoint
)
from std_msgs.msg import String
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer


# --- Combined Node: AutoCylinderEstimate + MarkerLanding ---
class AutoCylinderEstimateAndMarkerLanding(Node):
    def __init__(self):
        super().__init__('auto_cylinder_estimate_and_marker_landing')

        # QoS profiles for communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers for Offboard control, Trajectory setpoints, and Vehicle commands
        self.offb_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.traj_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.cmd_pub  = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Subscribers for Vehicle Odometry, Camera Info, and Marker Pose
        self.odom_sub = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_cb, qos_profile)
        self.caminfo_sub = self.create_subscription(CameraInfo, '/drone/front_depth/camera_info', self.caminfo_cb, 10)
        self.marker_pose_sub = self.create_subscription(VehicleOdometry, '/fmu/out/marker_pose', self.marker_pose_cb, qos_profile)

        # Subscribers for synchronized RGB and Depth images
        rgb_sub = Subscriber(self, Image, '/drone/front_rgb')
        depth_sub = Subscriber(self, Image, '/drone/front_depth')
        ats = ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=10, slop=0.1)
        ats.registerCallback(self.image_cb)

        # Internal states and parameters
        self.state = "ARM_OFFBOARD"
        self.position = [0.0, 0.0, 0.0]
        self.fx = self.fy = None
        self.marker_pose = [0.0, 0.0, 0.0]
        self.circle_radius = 15.0
        self.altitude = -5.0
        self.circle_speed = -0.02
        self.theta = 0.0
        self.start_theta = None
        self.offb_counter = 0
        self.takeoff_stage = 0
        self.one_rev_complete = False
        self.lower_hsv = np.array([0, 0, 110])
        self.upper_hsv = np.array([180, 40, 180])
        self.min_area = 5000
        self.bridge = CvBridge()

        cv2.namedWindow('Detection', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Mask', cv2.WINDOW_NORMAL)

        # Timer for periodic execution
        self.create_timer(0.1, self.timer_cb)

    def odom_cb(self, msg):
        self.position = [msg.position[0], msg.position[1], msg.position[2]]

    def caminfo_cb(self, msg):
        self.fx, self.fy = msg.k[0], msg.k[4]
        self.get_logger().info("Camera intrinsics received")
        self.destroy_subscription(self.caminfo_sub)

    def marker_pose_cb(self, msg):
        self.marker_pose = [msg.position[0], msg.position[1], msg.position[2]]

    def image_cb(self, rgb_msg, depth_msg):
        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough').astype(np.float32)
        depth[depth == 0] = np.nan

        if self.fx is None:
            cv2.imshow('Detection', rgb)
            cv2.waitKey(1)
            return

        hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
        cm = cv2.inRange(hsv, self.lower_hsv, self.upper_hsv) > 0
        dm = (depth > 1.0) & (depth < 30.0)
        mask = (cm & dm).astype(np.uint8) * 255
        mask_clean = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

        cnts, _ = cv2.findContours(mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        large = [c for c in cnts if cv2.contourArea(c) > self.min_area]

        overlay = rgb.copy()
        if large:
            c = max(large, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(c)
            roi = depth[y:y + h, x:x + w]
            vals = roi[np.isfinite(roi)]
            if vals.size > 0:
                Z = float(np.median(vals))
                w_m = (w * Z) / self.fx
                h_m = (h * Z) / self.fy

                cv2.rectangle(overlay, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(
                    overlay,
                    f"{w_m:.2f}m x {h_m:.2f}m",
                    (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2
                )
                self.get_logger().info(f"Detected cylinder: W={w_m:.2f} H={h_m:.2f} D={Z:.2f}")

        cv2.imshow('Detection', overlay)
        cv2.imshow('Mask', mask_clean)
        cv2.waitKey(1)

    def timer_cb(self):
        self.publish_offboard()

        # State machine logic for ARM, Takeoff, and Circle Navigation
        if self.offb_counter == 10:
            self.send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

        if self.state == "ARM_OFFBOARD":
            self.state = "ARM_TAKEOFF"

        elif self.state == "ARM_TAKEOFF":
            if self.takeoff_stage == 0:
                tz = self.altitude
                self.publish_setpoint(0, 0, tz, 0)
                if abs(self.position[2] - tz) < 0.5:
                    self.takeoff_stage = 1
            else:
                cx = self.circle_radius
                self.publish_setpoint(cx, 0, self.altitude, 0)
                if math.hypot(self.position[0] - cx, self.position[1]) < 0.5:
                    self.get_logger().info("Reached circle start → CIRCLE")
                    self.state = "CIRCLE"
                    self.start_theta = self.theta

        elif self.state == "CIRCLE":
            delta = abs(self.theta - self.start_theta)
            if delta >= 2 * math.pi:
                self.get_logger().info("Completed one revolution")
                self.one_rev_complete = True
                return

            x = self.circle_radius * math.cos(self.theta)
            y = self.circle_radius * math.sin(self.theta)
            z = self.altitude
            yaw = math.atan2(-y, -x)
            self.publish_setpoint(x, y, z, yaw)
            self.theta += self.circle_speed

        self.offb_counter += 1

    def publish_offboard(self):
        m = OffboardControlMode()
        m.position = True
        m.timestamp = self.get_clock().now().nanoseconds // 1000
        self.offb_pub.publish(m)

    def publish_setpoint(self, x, y, z, yaw):
        sp = TrajectorySetpoint()
        sp.position = [float(x), float(y), float(z)]
        sp.yaw = float(yaw)
        sp.timestamp = self.get_clock().now().nanoseconds // 1000
        self.traj_pub.publish(sp)

    def send_cmd(self, cmd, p1=0.0, p2=0.0):
        m = VehicleCommand()
        m.command = cmd
        m.param1 = float(p1)
        m.param2 = float(p2)
        m.target_system = 1
        m.target_component = 1
        m.source_system = 1
        m.source_component = 1
        m.from_external = True
        m.timestamp = self.get_clock().now().nanoseconds // 1000
        self.cmd_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = AutoCylinderEstimateAndMarkerLanding()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
