#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.pyplot as plt
import os
from datetime import datetime

class TrajectoryTracker(Node):
    def __init__(self):
        super().__init__('trajectory_tracker')

        self.home_dir = os.path.expanduser('~')
        self.plots_dir = os.path.join(self.home_dir, 'auv_ws', 'src', 'plots')
        os.makedirs(self.plots_dir, exist_ok=True)

        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.timestamps = []
        self.positions = []       # [x, y, z]
        self.orientations = []    # [roll, pitch, yaw] in degrees
        self.lin_vels = []        # [vx, vy, vz]
        self.ang_vels = []        # [wx, wy, wz] in deg/s

        self.start_time = None
        self.get_logger().info('Telemetry & Velocity Tracker Initialized.')

    def quat_to_euler(self, q):
        qw, qx, qy, qz = q.w, q.x, q.y, q.z
        
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)
        else:
            pitch = np.arcsin(sinp)

        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return np.degrees([roll, pitch, yaw])

    def imu_callback(self, msg: Imu):
        pass

    def odom_callback(self, msg: Odometry):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.start_time is None:
            self.start_time = now
        t = now - self.start_time

        pos = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        rpy = self.quat_to_euler(msg.pose.pose.orientation)

        lin_v = [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z]
        ang_v = np.degrees([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z])

        self.timestamps.append(t)
        self.positions.append(pos)
        self.orientations.append(rpy)
        self.lin_vels.append(lin_v)
        self.ang_vels.append(ang_v)

    def save_plots(self):
        if len(self.timestamps) < 2:
            print("[WARN] Insufficient telemetry data recorded.")
            return

        t = np.array(self.timestamps)
        pos = np.array(self.positions)
        rpy = np.array(self.orientations)
        lin = np.array(self.lin_vels)
        ang = np.array(self.ang_vels)

        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        # --- Plot 1: Standard Trajectory Analysis ---
        plot_path_traj = os.path.join(self.plots_dir, f'trajectory_{timestamp_str}.png')
        fig1 = plt.figure(figsize=(14, 10))
        fig1.suptitle('CSUR PJA Flight & Trajectory Analysis', fontsize=16, fontweight='bold')

        ax1 = fig1.add_subplot(2, 2, 1, projection='3d')
        ax1.plot(pos[:, 0], pos[:, 1], pos[:, 2], color='b', linewidth=1.5, label='Path')
        ax1.scatter(pos[0, 0], pos[0, 1], pos[0, 2], color='g', s=50, label='Start')
        ax1.scatter(pos[-1, 0], pos[-1, 1], pos[-1, 2], color='r', s=50, label='End')
        ax1.set_title('3D Trajectory (m)')
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_zlabel('Z (m)')
        ax1.legend()

        ax2 = fig1.add_subplot(2, 2, 2)
        ax2.plot(t, pos[:, 2], color='cyan', linewidth=2, label='Z Position')
        ax2.axhline(y=0.5, color='r', linestyle='--', label='Target Altitude (0.5m)')
        ax2.set_title('Altitude Profile (Z vs Time)')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Z (m)')
        ax2.grid(True)
        ax2.legend()

        ax3 = fig1.add_subplot(2, 2, 3)
        ax3.plot(t, rpy[:, 0], label='Roll (°)', color='r')
        ax3.plot(t, rpy[:, 1], label='Pitch (°)', color='g')
        ax3.plot(t, rpy[:, 2], label='Yaw (°)', color='b')
        ax3.set_title('Orientation Attitude (Degrees)')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Angle (°)')
        ax3.grid(True)
        ax3.legend()

        ax4 = fig1.add_subplot(2, 2, 4)
        ax4.plot(pos[:, 0], pos[:, 1], color='magenta', linewidth=1.5)
        ax4.set_title('Planar Motion (X vs Y)')
        ax4.set_xlabel('X (m)')
        ax4.set_ylabel('Y (m)')
        ax4.grid(True)

        plt.tight_layout()
        plt.savefig(plot_path_traj, dpi=300)
        plt.close(fig1)

        # --- Plot 2: Linear & Angular Velocity Diagnostics ---
        plot_path_vel = os.path.join(self.plots_dir, f'velocity_{timestamp_str}.png')
        fig2, (ax_lin, ax_ang) = plt.subplots(2, 1, figsize=(14, 10))
        fig2.suptitle('CSUR PJA Dynamic Velocity Diagnostics', fontsize=16, fontweight='bold')

        # Subplot 1: Linear Velocities
        ax_lin.plot(t, lin[:, 0], color='r', label='Vx (Surge)')
        ax_lin.plot(t, lin[:, 1], color='g', label='Vy (Sway)')
        ax_lin.plot(t, lin[:, 2], color='b', label='Vz (Heave)')
        ax_lin.set_title('Linear Velocities (m/s)')
        ax_lin.set_xlabel('Time (s)')
        ax_lin.set_ylabel('Velocity (m/s)')
        ax_lin.grid(True)
        ax_lin.legend()

        # Subplot 2: Angular Velocities
        ax_ang.plot(t, ang[:, 0], color='r', linestyle='--', label='Wx (Roll Rate)')
        ax_ang.plot(t, ang[:, 1], color='g', linestyle='--', label='Wy (Pitch Rate)')
        ax_ang.plot(t, ang[:, 2], color='b', label='Wz (Yaw Rate)')
        ax_ang.set_title('Angular Velocities (deg/s)')
        ax_ang.set_xlabel('Time (s)')
        ax_ang.set_ylabel('Angular Rate (°/s)')
        ax_ang.grid(True)
        ax_ang.legend()

        plt.tight_layout()
        plt.savefig(plot_path_vel, dpi=300)
        plt.close(fig2)

        print(f"\n[SUCCESS] Trajectory Plot: {plot_path_traj}")
        print(f"[SUCCESS] Velocity Plot:   {plot_path_vel}")

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_plots()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()