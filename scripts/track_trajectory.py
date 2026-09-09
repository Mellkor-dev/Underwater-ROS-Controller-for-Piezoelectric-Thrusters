#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.pyplot as plt
import os
import sys
from datetime import datetime

class TrajectoryTracker(Node):
    def __init__(self):
        super().__init__('trajectory_tracker')

        # Directories setup
        self.home_dir = os.path.expanduser('~')
        self.plots_dir = os.path.join(self.home_dir, 'auv_ws', 'src', 'plots')
        os.makedirs(self.plots_dir, exist_ok=True)

        # Topic Subscriptions
        self.imu_sub = self.create_subscription(
            Imu, '/imu', self.imu_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        # Data Storage
        self.timestamps = []
        self.positions = []  # [x, y, z]
        self.orientations = []  # [roll, pitch, yaw] in degrees
        self.accels = []  # [ax, ay, az]

        self.start_time = None
        self.get_logger().info('Trajectory Tracker Initialized. Recording telemetry...')
        self.get_logger().info(f'Plots will be saved to: {self.plots_dir}')

    def quat_to_euler(self, q):
        qw, qx, qy, qz = q.w, q.x, q.y, q.z
        
        # Roll (X-axis)
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (Y-axis)
        sinp = 2 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)
        else:
            pitch = np.arcsin(sinp)

        # Yaw (Z-axis)
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return np.degrees([roll, pitch, yaw])

    def imu_callback(self, msg: Imu):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.start_time is None:
            self.start_time = now
        t = now - self.start_time

        rpy = self.quat_to_euler(msg.orientation)
        accel = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]

        # Log IMU orientation state if odometry is unavailable
        if len(self.positions) == 0:
            self.timestamps.append(t)
            self.positions.append([0.0, 0.0, 0.5])  # Default spawn coordinate
            self.orientations.append(rpy)
            self.accels.append(accel)

    def odom_callback(self, msg: Odometry):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.start_time is None:
            self.start_time = now
        t = now - self.start_time

        pos = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        rpy = self.quat_to_euler(msg.pose.pose.orientation)

        self.timestamps.append(t)
        self.positions.append(pos)
        self.orientations.append(rpy)

    def save_plots(self):
        if len(self.timestamps) < 2:
            print("[WARN] Insufficient telemetry data recorded.")
            return

        t = np.array(self.timestamps)
        pos = np.array(self.positions)
        rpy = np.array(self.orientations)

        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        plot_path = os.path.join(self.plots_dir, f'trajectory_{timestamp_str}.png')

        fig = plt.figure(figsize=(14, 10))
        fig.suptitle('CSUR PJA Flight & Trajectory Analysis', fontsize=16, fontweight='bold')

        # Subplot 1: 3D Trajectory
        ax1 = fig.add_subplot(2, 2, 1, projection='3d')
        ax1.plot(pos[:, 0], pos[:, 1], pos[:, 2], color='b', linewidth=1.5, label='Path')
        ax1.scatter(pos[0, 0], pos[0, 1], pos[0, 2], color='g', s=50, label='Start')
        ax1.scatter(pos[-1, 0], pos[-1, 1], pos[-1, 2], color='r', s=50, label='End')
        ax1.set_title('3D Trajectory (m)')
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_zlabel('Z (m)')
        ax1.legend()

        # Subplot 2: Altitude Profile with Fixed Limits
        ax2 = fig.add_subplot(2, 2, 2)
        ax2.plot(t, pos[:, 2], color='cyan', linewidth=2, label='Z Position')
        ax2.axhline(y=0.5, color='r', linestyle='--', label='Target Altitude (0.5m)')
        ax2.set_title('Altitude Profile (Z vs Time)')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Z (m)')
        ax2.set_ylim([0.0, 1.0])  # Locked y-axis range to show true 1m tank depth context
        ax2.grid(True)
        ax2.legend()

        # Subplot 3: Orientation Attitude with Fixed Limits
        ax3 = fig.add_subplot(2, 2, 3)
        ax3.plot(t, rpy[:, 0], label='Roll (°)', color='r')
        ax3.plot(t, rpy[:, 1], label='Pitch (°)', color='g')
        ax3.plot(t, rpy[:, 2], label='Yaw (°)', color='b')
        ax3.set_title('Orientation Attitude (Degrees)')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Angle (°)')
        ax3.set_ylim([-15.0, 15.0])  # Locked angle range
        ax3.grid(True)
        ax3.legend()

        # Subplot 4: Planar Motion
        ax4 = fig.add_subplot(2, 2, 4)
        ax4.plot(pos[:, 0], pos[:, 1], color='magenta', linewidth=1.5)
        ax4.set_title('Planar Motion (X vs Y)')
        ax4.set_xlabel('X (m)')
        ax4.set_ylabel('Y (m)')
        ax4.grid(True)

        plt.tight_layout()
        plt.savefig(plot_path, dpi=300)
        plt.close()

        # Plain print avoids ROS 2 logger context crash during node shutdown
        print(f"\n[SUCCESS] Plots generated and saved to: {plot_path}")
def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Stopping recorder and rendering plots...')
    finally:
        node.save_plots()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()