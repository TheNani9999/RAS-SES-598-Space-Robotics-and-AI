import math
import time
import re
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.logging import LoggingSeverity

from px4_msgs.msg import VehicleOdometry, OffboardControlMode, VehicleCommand, TrajectorySetpoint
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# === PARAMETERS ===
K_LAT = 0.1        # Lateral correction gain (x, y)
K_ALT = 0.05       # Altitude correction gain
ERROR_THRESHOLD = 0.1
DESIRED_ALT = 0.0  # Desired landing altitude

# === MAIN CLASS ===
class MarkerLanding(Node):
    def __init__(self):
        super().__init__('marker_landing_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        self.odom_sub = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_cb, qos_profile)
        self.marker_pose_sub = self.create_subscription(String, '/aruco/marker_pose', self.marker_pose_cb, 10)
        self.down_mono_sub = self.create_subscription(Image, '/drone/down_mono', self.down_mono_callback, 10)
        self.debug_image_pub = self.create_publisher(Image, '/aruco/debug_image', 10)

        self.cv_bridge = CvBridge()

        self.state = "WAIT"
        self.offboard_counter = 0
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.position = [0.0, 0.0, -13.0]

        self.marker1_sample = None
        self.marker1_sample_time = None
        self.marker2_sample = None
        self.marker2_sample_time = None

        self.marker1_pos = None
        self.marker2_pos = None

        self.detect_start_time = None
        self.return_start_time = None

        self.chosen_marker_pos = None
        self.chosen_direction_y = None
        self.latest_marker_pose = None

    def odom_cb(self, msg: VehicleOdometry):
        self.position = [msg.position[0], msg.position[1], msg.position[2]]

    def marker_pose_cb(self, msg: String):
        line = msg.data.strip()
        match = re.search(r"x:\s*([\-\d.]+).*y:\s*([\-\d.]+).*z:\s*([\-\d.]+)", line)
        if match:
            x, y, z = float(match.group(1)), float(match.group(2)), float(match.group(3))
            self.latest_marker_pose = (x, y, z)

            if self.state == "DETECT_FIRST":
                self.marker1_sample = (x, y, z)
                self.marker1_sample_time = time.time()

            elif self.state == "DETECT_SECOND":
                self.marker2_sample = (x, y, z)
                self.marker2_sample_time = time.time()

    def down_mono_callback(self, msg: Image):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            self.get_logger().error("Failed to convert down mono image")
            return

        text = f"Drone Pos: x:{self.position[0]:.2f}, y:{self.position[1]:.2f}, z:{self.position[2]:.2f}"
        cv2.putText(cv_image, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255,255,255), 2)
        debug_msg = self.cv_bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        debug_msg.header.stamp = msg.header.stamp
        self.debug_image_pub.publish(debug_msg)

    def timer_callback(self):
        self.publish_offboard_control_mode()
        if self.offboard_counter == 5 and self.state not in ("WAIT", "DONE"):
            self.engage_offboard_mode()
            self.arm()

        if self.state == "WAIT":
            self.state = "ARM_TAKEOFF"

        elif self.state == "ARM_TAKEOFF":
            self._goto([0.0, 0.0, -13.0], "GOTO_FIRST")

        elif self.state == "GOTO_FIRST":
            self._goto([0.0, 5.0, -13.0], "DETECT_FIRST", start_detection=True)

        elif self.state == "DETECT_FIRST":
            self._detect_marker("marker1", "RETURN_CENTER_FIRST")

        elif self.state == "RETURN_CENTER_FIRST":
            self._return_to_center("GOTO_SECOND")

        elif self.state == "GOTO_SECOND":
            self._goto([0.0, -5.0, -13.0], "DETECT_SECOND", start_detection=True)

        elif self.state == "DETECT_SECOND":
            self._detect_marker("marker2", "RETURN_CENTER_SECOND")

        elif self.state == "RETURN_CENTER_SECOND":
            self._return_to_center("COMPARE")

        elif self.state == "COMPARE":
            d1 = math.sqrt(sum([(a - b)**2 for a, b in zip(self.marker1_pos, self.position)])) if self.marker1_pos else float('inf')
            d2 = math.sqrt(sum([(a - b)**2 for a, b in zip(self.marker2_pos, self.position)])) if self.marker2_pos else float('inf')

            if d1 < d2:
                self.chosen_marker_pos = self.marker1_pos
                self.chosen_direction_y = 5.0
            else:
                self.chosen_marker_pos = self.marker2_pos
                self.chosen_direction_y = -5.0

            self.state = "APPROACH"

        elif self.state == "APPROACH":
            target = [0.0, self.chosen_direction_y, -13.0]
            self._goto(target, "LAND_IBVS")

        elif self.state == "LAND_IBVS":
            if self.latest_marker_pose is None:
                self.publish_trajectory_setpoint(*self.position)
                return

            dx = -self.latest_marker_pose[0]
            dy = -self.latest_marker_pose[1]
            dz = self.position[2] - DESIRED_ALT

            if abs(dx) < ERROR_THRESHOLD and abs(dy) < ERROR_THRESHOLD and abs(dz) < ERROR_THRESHOLD:
                self.disarm()
                self.state = "DONE"
            else:
                new_x = self.position[0] + K_LAT * dx
                new_y = self.position[1] + K_LAT * dy
                new_z = self.position[2] - K_ALT * dz
                self.publish_trajectory_setpoint(new_x, new_y, new_z)

        elif self.state == "DONE":
            self.publish_trajectory_setpoint(*self.position)

    def _goto(self, target, next_state, start_detection=False):
        self.publish_trajectory_setpoint(*target)
        if self._dist_to(target) < 0.5:
            if start_detection:
                self.detect_start_time = time.time()
            self.state = next_state

    def _detect_marker(self, marker_id, next_state):
        elapsed = time.time() - self.detect_start_time
        marker_sample = getattr(self, f"{marker_id}_sample")
        marker_pos = getattr(self, f"{marker_id}_pos")

        self.publish_trajectory_setpoint(*self.position)
        if elapsed >= 4.0 and marker_sample and marker_pos is None:
            setattr(self, f"{marker_id}_pos", marker_sample)
        if elapsed >= 5.0:
            if marker_pos is None:
                setattr(self, f"{marker_id}_pos", (999.0, 999.0, 999.0))
            self.return_start_time = time.time()
            self.state = next_state

    def _return_to_center(self, next_state):
        self.publish_trajectory_setpoint(0.0, 0.0, -13.0)
        if time.time() - self.return_start_time >= 2.0:
            self.state = next_state

    def _dist_to(self, target):
        dx = self.position[0] - target[0]
        dy = self.position[1] - target[1]
        dz = self.position[2] - target[2]
        return math.sqrt(dx**2 + dy**2 + dz**2)

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(msg)
        self.offboard_counter += 1

    def publish_trajectory_setpoint(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_pub.publish(msg)

    def arm(self):
        self._publish_vehicle_command(400, 1)

    def disarm(self):
        self._publish_vehicle_command(400, 0)

    def engage_offboard_mode(self):
        self._publish_vehicle_command(92, 1)

    def _publish_vehicle_command(self, command, param1):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(param1)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_cmd_pub.publish(msg)

# === MAIN EXECUTION ===
def main(args=None):
    rclpy.init(args=args)
    node = MarkerLanding()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
