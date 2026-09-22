#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np

class DynamicDepthHoldPID(Node):
    def __init__(self):
        super().__init__('depth_hold_pid')

        # Control Publishers & Subscriptions
        # Publish ONLY to /cmd_vel (pja_controller listens here)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Subscribe ONLY to waypoint_tracker guidance output
        self.guidance_sub = self.create_subscription(Twist, '/teleop/cmd_vel', self.guidance_callback, 10)

        # PID Gains for Heave (Z)
        self.Kp = 3.5
        self.Ki = 0.15
        self.Kd = 0.8

        self.target_z = 0.5  # Target operational depth (0.5m)
        self.current_z = 0.0
        self.current_vz = 0.0
        self.integral_error = 0.0

        # Guidance passthrough attributes
        self.guidance_surge = 0.0
        self.guidance_yaw = 0.0
        self.last_guidance_time = self.get_clock().now()
        self.odom_received = False
        self.dt = 0.05  # 20 Hz loop rate
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info("Dynamic Altitude Hold PID Active (Enforcing Z = 0.5m during transit).")

    def odom_callback(self, msg: Odometry):
        self.current_z = msg.pose.pose.position.z
        self.current_vz = msg.twist.twist.linear.z
        self.odom_received = True

    def guidance_callback(self, msg: Twist):
        # Read guidance commands coming strictly from waypoint_tracker
        self.guidance_surge = msg.linear.x
        self.guidance_yaw = msg.angular.z
        self.last_guidance_time = self.get_clock().now()
    def control_loop(self):
        if not self.odom_received:
            return

        if (self.get_clock().now() - self.last_guidance_time).nanoseconds > 0.5e9:
            self.guidance_surge = 0.0
            self.guidance_yaw = 0.0
        # Closed-Loop Depth Control
        error = self.target_z - self.current_z
        u_ff = 0.3  # Baseline buoyancy compensation

        P = self.Kp * error
        self.integral_error = np.clip(self.integral_error + error * self.dt, -0.4, 0.4)
        I = self.Ki * self.integral_error
        
        # D-term: damp velocity towards target depth
        D = -self.Kd * self.current_vz

        u_heave = float(np.clip(u_ff + P + I + D, 0.0, 1.0))

        # Forward guidance values to pja_controller
        cmd = Twist()
        cmd.linear.x = float(self.guidance_surge)
        cmd.angular.z = float(self.guidance_yaw)
        cmd.linear.z = u_heave

        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = DynamicDepthHoldPID()
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