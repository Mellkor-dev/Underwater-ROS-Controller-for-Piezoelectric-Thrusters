#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np

class WaypointTracker(Node):
    def __init__(self):
        super().__init__('waypoint_tracker')

        self.cmd_pub = self.create_publisher(Twist, '/teleop/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # 5 Target Waypoints [X, Y, Z] in meters
        self.waypoints = np.array([
            [1.0,  0.0, 0.5],
            [2.0,  0.5, 0.5],
            [3.0, -0.5, 0.5],
            [4.0,  0.0, 0.5],
            [5.0,  0.0, 0.5]
        ])
        self.current_idx = 0
        self.goal_tolerance = 0.25  # Relaxed acceptance radius to 25cm

        self.current_pose = np.zeros(3)
        self.current_yaw = 0.0
        self.odom_received = False

        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz
        self.get_logger().info('5-Waypoint Line-of-Sight Guidance Active.')

    def odom_callback(self, msg: Odometry):
        self.current_pose[0] = msg.pose.pose.position.x
        self.current_pose[1] = msg.pose.pose.position.y
        self.current_pose[2] = msg.pose.pose.position.z

        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = np.arctan2(siny_cosp, cosy_cosp)
        self.odom_received = True

    def control_loop(self):
        if not self.odom_received or self.current_idx >= len(self.waypoints):
            if self.current_idx >= len(self.waypoints):
                self.get_logger().info('Mission Complete. Holding position.')
                self.cmd_pub.publish(Twist())  # Stop at end of route
            return

        target = self.waypoints[self.current_idx]
        error_vec = target - self.current_pose
        dist_2d = np.linalg.norm(error_vec[:2])

        # Waypoint Advancement Check: Distance sphere or passed-by check
        if dist_2d < self.goal_tolerance or (self.current_pose[0] > target[0] and abs(error_vec[1]) < 0.3):
            self.get_logger().info(f'---> REACHED WAYPOINT {self.current_idx + 1}: {target}')
            self.current_idx += 1
            return

        target_yaw = np.arctan2(error_vec[1], error_vec[0])
        yaw_error = np.arctan2(np.sin(target_yaw - self.current_yaw), np.cos(target_yaw - self.current_yaw))

        cmd = Twist()
        # Absolute speed caps prevent runaway linear acceleration
        cmd.linear.x = float(np.clip(0.12 * dist_2d, 0.03, 0.12))       # Fixed speed ceiling 0.12 m/s
        cmd.angular.z = float(np.clip(-0.3 * yaw_error, -0.15, 0.15))    # Active heading correction
        cmd.linear.z = float(np.clip(0.3 * error_vec[2], -0.05, 0.05))   # Depth lock

        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointTracker()
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