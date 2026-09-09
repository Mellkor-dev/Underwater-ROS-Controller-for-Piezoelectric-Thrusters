#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np

class HoldCurrentZPID(Node):
    def __init__(self):
        super().__init__('depth_hold_pid')

        # Output to PJA allocation matrix
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Telemetry & Teleop Subscriptions
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Twist, '/teleop/cmd_vel', self.teleop_callback, 10)

        # Dynamic State Variables
        self.current_z = 0.0
        self.current_vz = 0.0
        self.target_z = None  # Latched setpoint (sampled on idle)
        self.odom_received = False

        # Teleop Watchdog
        self.last_teleop_time = self.get_clock().now()
        self.teleop_active = False
        self.manual_cmd = Twist()

        # PID Gains for CSUR 85g Mass
        self.Kp = 0.8
        self.Ki = 0.02
        self.Kd = 0.25
        self.integral_error = 0.0
        self.dt = 0.02

        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info('Dynamic Altitude Hold PID Active (Holding Current Z on Idle)')

    def odom_callback(self, msg: Odometry):
        self.current_z = msg.pose.pose.position.z
        self.current_vz = msg.twist.twist.linear.z
        self.odom_received = True

    def teleop_callback(self, msg: Twist):
        self.manual_cmd = msg
        self.last_teleop_time = self.get_clock().now()
        
        # Check for active user command
        is_user_input = (abs(msg.linear.x) > 1e-3 or abs(msg.linear.z) > 1e-3 or 
                         abs(msg.angular.z) > 1e-3)
        
        if is_user_input:
            self.teleop_active = True
            # Clear target so it re-latches the moment user releases keys
            self.target_z = None
        else:
            if self.teleop_active:
                # User just released keys: latch exact current Z
                self.target_z = self.current_z
                self.get_logger().info(f"Manual input released. Latched holding altitude: Z = {self.target_z:.3f} m")
            self.teleop_active = False

    def control_loop(self):
        if not self.odom_received:
            return

        dt_teleop = (self.get_clock().now() - self.last_teleop_time).nanoseconds * 1e-9
        
        # Watchdog: If no teleop input for 0.3s, transition to position-hold
        if dt_teleop > 0.3 and self.teleop_active:
            self.teleop_active = False
            self.target_z = self.current_z
            self.get_logger().info(f"Teleop timeout. Auto-latched holding altitude: Z = {self.target_z:.3f} m")

        # Initial launch case: If idle at spawn, latch current Z immediately
        if self.target_z is None and not self.teleop_active:
            self.target_z = self.current_z
            self.get_logger().info(f"Initial spawn altitude latched: Z = {self.target_z:.3f} m")

        if self.teleop_active:
            # Pass user command directly
            self.cmd_pub.publish(self.manual_cmd)
            self.integral_error = 0.0
        else:
            # PID Closed-Loop Hold around latched target_z
            # Position error in World Z-frame
            error = self.target_z - self.current_z

            P = self.Kp * error
            self.integral_error += error * self.dt
            self.integral_error = np.clip(self.integral_error, -0.1, 0.1)
            I = self.Ki * self.integral_error
            D = -self.Kd * self.current_vz

            # Output range clamped [-0.3, 0.3]
            u_heave = float(np.clip(P + I + D, -0.3, 0.3))

            
            cmd = Twist()
            cmd.linear.z = u_heave
            self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = HoldCurrentZPID()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()