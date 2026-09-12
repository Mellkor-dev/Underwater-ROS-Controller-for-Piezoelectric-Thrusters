#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from ros_gz_interfaces.msg import Entity, EntityWrench
from tf2_ros import TransformBroadcaster
import numpy as np

class PJAControllerOriented(Node):
    def __init__(self):
        super().__init__('pja_controller_oriented')

        self.wrench_pub = self.create_publisher(
            EntityWrench, '/world/underwater_world/wrench/persistent', 10
        )
        self.clear_pub = self.create_publisher(
            Entity, '/world/underwater_world/wrench/clear', 10
        )

        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu', self.imu_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        # TF Broadcaster for RViz Odometry Visualization
        self.tf_broadcaster = TransformBroadcaster(self)

        self.update_rate = 50.0  # Hz
        self.timer = self.create_timer(1.0 / self.update_rate, self.control_loop)

        self.cmd_linear = [0.0, 0.0, 0.0]
        self.cmd_angular = [0.0, 0.0, 0.0]
        self.last_cmd_time = self.get_clock().now()
        self.cmd_timeout = 0.5

        self.q = [1.0, 0.0, 0.0, 0.0]
        self.target_entity_id = 13 

        self.pja_offsets = {
            'hover_front':  {'x':  0.03828, 'y':  0.00000, 'z': -0.00520, 'dir': [0, 0, 1]},
            'hover_rear':   {'x': -0.03828, 'y':  0.00000, 'z': -0.00520, 'dir': [0, 0, 1]},
            'left_jet_f':   {'x': -0.00870, 'y':  0.03848, 'z':  0.00350, 'dir': [1, 0, 0]},
            'right_jet_f':  {'x': -0.00870, 'y': -0.03848, 'z':  0.00350, 'dir': [1, 0, 0]},
            'left_jet_r':   {'x':  0.00870, 'y':  0.03848, 'z':  0.00350, 'dir': [-1, 0, 0]},
            'right_jet_r':  {'x':  0.00870, 'y': -0.03848, 'z':  0.00350, 'dir': [-1, 0, 0]},
        }

        self.max_thrust = 0.05  # N (Peak PJA force)

        self.get_logger().info('PJA Allocation Controller updated with TF Broadcaster & True ICR.')

    def imu_callback(self, msg: Imu):
        self.q = [msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z]

    def odom_callback(self, msg: Odometry):
        # Broadcast TF frame from odom -> base_footprint for RViz visualization
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

    def cmd_vel_callback(self, msg: Twist):
        self.cmd_linear = [msg.linear.x, msg.linear.y, msg.linear.z]
        self.cmd_angular = [msg.angular.x, msg.angular.y, msg.angular.z]
        self.last_cmd_time = self.get_clock().now()

    def quat_to_rot_matrix(self, q):
        qw, qx, qy, qz = q
        return np.array([
            [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)],
            [2*(qx*qy + qw*qz), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qw*qx)],
            [2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx**2 + qy**2)]
        ])

    def control_loop(self):
        dt_cmd = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if dt_cmd > self.cmd_timeout:
            self.cmd_linear = [0.0, 0.0, 0.0]
            self.cmd_angular = [0.0, 0.0, 0.0]

        u_surge, u_heave = self.cmd_linear[0], self.cmd_linear[2]
        u_pitch, u_yaw = self.cmd_angular[1], self.cmd_angular[2]

        # ICR Differential Arc Turn Mapping
        # Reduces yaw sensitivity (0.2 factor) so w+d maintains forward momentum while turning
        yaw_gain = 0.2 * u_yaw

        demands = {
            'hover_front': max(0.0, u_heave + u_pitch),
            'hover_rear':  max(0.0, u_heave - u_pitch),
            
            # Differential forward thrust for smooth arc turns
            'left_jet_f':  max(0.0, u_surge + yaw_gain),
            'right_jet_f': max(0.0, u_surge - yaw_gain),
            
            'left_jet_r':  max(0.0, -u_surge - yaw_gain),
            'right_jet_r': max(0.0, -u_surge + yaw_gain),
        }

        any_active = any(demand >= 1e-4 for demand in demands.values())

        if not any_active:
            clear_msg = Entity()
            clear_msg.name = 'Centroid_body::base_footprint'
            clear_msg.id = self.target_entity_id
            clear_msg.type = 3
            self.clear_pub.publish(clear_msg)
            return

        body_fx, body_fy, body_fz = 0.0, 0.0, 0.0
        body_tx, body_ty, body_tz = 0.0, 0.0, 0.0

        for name, demand in demands.items():
            if demand < 1e-4:
                continue

            intensity = min(1.0, demand)
            thrust_mag = intensity * self.max_thrust

            pja = self.pja_offsets[name]
            fx, fy, fz = pja['dir'][0] * thrust_mag, pja['dir'][1] * thrust_mag, pja['dir'][2] * thrust_mag
            rx, ry, rz = pja['x'], pja['y'], pja['z']

            body_fx += fx
            body_fy += fy
            body_fz += fz
            body_tx += ry * fz - rz * fy
            body_ty += rz * fx - rx * fz
            body_tz += rx * fy - ry * fx

        # Convert body-frame forces to world-frame forces using active IMU orientation
        R = self.quat_to_rot_matrix(self.q)
        world_force = R @ np.array([body_fx, body_fy, body_fz])
        world_torque = R @ np.array([body_tx, body_ty, body_tz])

        wrench_msg = EntityWrench()
        wrench_msg.entity.name = 'Centroid_body::base_footprint'
        wrench_msg.entity.id = self.target_entity_id
        wrench_msg.entity.type = 3

        wrench_msg.wrench.force.x = float(world_force[0])
        wrench_msg.wrench.force.y = float(world_force[1])
        wrench_msg.wrench.force.z = float(world_force[2])

        wrench_msg.wrench.torque.x = float(world_torque[0])
        wrench_msg.wrench.torque.y = float(world_torque[1])
        wrench_msg.wrench.torque.z = float(world_torque[2])

        self.wrench_pub.publish(wrench_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PJAControllerOriented()
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