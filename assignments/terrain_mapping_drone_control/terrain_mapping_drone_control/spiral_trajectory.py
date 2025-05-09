
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import math
import time

# ROS 2 messages
from px4_msgs.msg import (
    VehicleOdometry, OffboardControlMode, VehicleCommand, VehicleStatus, TrajectorySetpoint
)
from std_msgs.msg import Float64

class SpiralToHover(Node):
    def __init__(self):
        super().__init__('spiral_to_hover')

        # QoS setup
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.gimbal_pitch_pub = self.create_publisher(Float64, '/model/x500_gimbal_0/command/gimbal_pitch', 10)
        self.gimbal_roll_pub  = self.create_publisher(Float64, '/model/x500_gimbal_0/command/gimbal_roll', 10)
        self.gimbal_yaw_pub   = self.create_publisher(Float64, '/model/x500_gimbal_0/command/gimbal_yaw', 10)

        self.vehicle_odometry_sub = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)
        self.vehicle_status_sub   = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        self.vehicle_odometry = VehicleOdometry()
        self.vehicle_status = VehicleStatus()
        self.offboard_setpoint_counter = 0

        self.start_time = time.time()
        self.loop_start_time = None
        self.hover_start_time = None
        self.hover_phase = None
        self.hover_timer = None

        # Flight configuration
        self.altitude_steps = [2.0, 4.0, 6.0, 8.0, 10.0, 12.0]
        self.current_loop = 0
        self.loop_duration = 20.0
        self.RADIUS = 10.0
        self.climb_altitude = 14.0

        self.state = "TAKEOFF"
        self.create_timer(0.05, self.timer_callback)

    def vehicle_odometry_callback(self, msg):
        self.vehicle_odometry = msg
        self.update_gimbal()

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg

    def update_gimbal(self):
        try:
            x = self.vehicle_odometry.position[0]
            y = self.vehicle_odometry.position[1]
            yaw = math.atan2(-y, -x)
        except Exception:
            yaw = 0.0

        msg = Float64()
        msg.data = yaw
        self.gimbal_yaw_pub.publish(msg)

        msg.data = 0.0
        self.gimbal_pitch_pub.publish(msg)
        self.gimbal_roll_pub.publish(msg)


    def arm(self):
        cmd = VehicleCommand()
        cmd.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        cmd.param1 = 1.0
        cmd = self._fill_command_header(cmd)
        self.vehicle_command_pub.publish(cmd)
        self.get_logger().info("Arm command sent")

    def engage_offboard(self):
        cmd = VehicleCommand()
        cmd.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        cmd.param1 = 1.0
        cmd.param2 = 6.0
        cmd = self._fill_command_header(cmd)
        self.vehicle_command_pub.publish(cmd)
        self.get_logger().info("Offboard mode command sent")

    def _fill_command_header(self, cmd):
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        return cmd

    def publish_offboard(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(msg)

    def publish_setpoint(self, x, y, z, yaw):
        sp = TrajectorySetpoint()
        sp.position = [x, y, z]
        sp.yaw = yaw
        sp.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_pub.publish(sp)

 
    def is_at_altitude(self, target):
        try:
            current = -self.vehicle_odometry.position[2]
            return abs(current - target) < 0.5
        except Exception:
            return False

    def calculate_circle(self, t, target_alt):
        omega = 2 * math.pi / self.loop_duration
        x = self.RADIUS * math.cos(omega * t)
        y = self.RADIUS * math.sin(omega * t)
        z = -target_alt
        yaw = math.atan2(-y, -x)
        return x, y, z, yaw


    def timer_callback(self):
        if self.offboard_setpoint_counter == 10:
            self.engage_offboard()
            self.arm()
            self.start_time = time.time()
            self.loop_start_time = self.start_time

        self.publish_offboard()

        if self.state == "TAKEOFF":
            target = self.altitude_steps[0]
            if not self.is_at_altitude(target):
                self.publish_setpoint(0.0, 0.0, -target, 0.0)
                alt = -self.vehicle_odometry.position[2] if hasattr(self.vehicle_odometry, "position") else 0.0
                self.get_logger().info(f"Taking off... Altitude: {alt:.2f} m (target: {target} m)")
            else:
                self.state = "LOOP"
                self.loop_start_time = time.time()
                self.get_logger().info("Takeoff complete. Starting loops.")

        elif self.state == "LOOP":
            t = time.time() - self.loop_start_time
            target_alt = self.altitude_steps[self.current_loop]
            x, y, z, yaw = self.calculate_circle(t, target_alt)
            self.publish_setpoint(x, y, z, yaw)

            self.get_logger().info(f"Loop {self.current_loop+1}: pos=({x:.2f}, {y:.2f}), alt={target_alt:.2f} m, yaw={yaw:.2f}")
            if t > self.loop_duration:
                self.current_loop += 1
                self.loop_start_time = time.time()
                if self.current_loop < len(self.altitude_steps):
                    self.get_logger().info(f"Starting loop {self.current_loop+1}")
                else:
                    self.state = "RETURN"
                    self.get_logger().info("Loops complete. Returning to origin.")


        elif self.state == "RETURN":
            target_alt = self.altitude_steps[-1]
            self.publish_setpoint(0.0, 0.0, -target_alt, 0.0)
            x = self.vehicle_odometry.position[0]
            y = self.vehicle_odometry.position[1]
            dist = math.sqrt(x ** 2 + y ** 2)
            self.get_logger().info(f"Returning: distance={dist:.2f} m")
            if dist < 1.0:
                self.state = "CLIMB"
                self.get_logger().info("Origin reached. Climbing.")

     
        elif self.state == "CLIMB":
            self.publish_setpoint(0.0, 0.0, -self.climb_altitude, 0.0)
            if self.is_at_altitude(self.climb_altitude):
                self.state = "HOVER"
                self.hover_phase = "first_side"
                self.hover_timer = time.time()
                self.get_logger().info("Climb complete. Hovering with motion.")
            else:
                self.get_logger().info("Climbing...")

    
        elif self.state == "HOVER":
            if self.hover_phase == "first_side":
                self.publish_setpoint(0.0, 3.0, -self.climb_altitude, 0.0)
                if time.time() - self.hover_timer > 5:
                    self.hover_phase = "second_side"
                    self.hover_timer = time.time()
                    self.get_logger().info("Switching to second hover side")
            elif self.hover_phase == "second_side":
                self.publish_setpoint(0.0, -3.0, -self.climb_altitude, 0.0)

        self.offboard_setpoint_counter += 1

def main():
    rclpy.init()
    node = SpiralToHover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Flight interrupted.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
