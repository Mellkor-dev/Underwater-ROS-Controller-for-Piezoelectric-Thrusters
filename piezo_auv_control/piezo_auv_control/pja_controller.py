#!/usr/bin/env python3
import rclpy
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from ros_gz_interfaces.msg import Entity, EntityWrench
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import numpy as np

class PJAControllerOriented(Node):
    def __init__(self):
        super().__init__('pja_controller_oriented')

        # Reverted back to persistent wrench publisher
        self.wrench_pub = self.create_publisher(
            EntityWrench, '/world/underwater_world/wrench', 10
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

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_map_to_odom_static()

        self.update_rate = 50.0  # Hz
        self.timer = self.create_timer(1.0 / self.update_rate, self.control_loop)

        self.cmd_linear = [0.0, 0.0, 0.0]
        self.cmd_angular = [0.0, 0.0, 0.0]
        self.last_cmd_time = self.get_clock().now()
        self.cmd_timeout = 0.5

        self.q = [1.0, 0.0, 0.0, 0.0]
        self.model_ready = False
        self.wrench_currently_active = False

        self.pja_offsets = {
            'hover_front':  {'x':  0.03828, 'y':  0.00000, 'z': -0.00520, 'dir': [0, 0, 1]},
            'hover_rear':   {'x': -0.03828, 'y':  0.00000, 'z': -0.00520, 'dir': [0, 0, 1]},
            'left_jet_f':   {'x': -0.00870, 'y':  0.03848, 'z':  0.00350, 'dir': [1, 0, 0]},
            'right_jet_f':  {'x': -0.00870, 'y': -0.03848, 'z':  0.00350, 'dir': [1, 0, 0]},
            'left_jet_r':   {'x':  0.00870, 'y':  0.03848, 'z':  0.00350, 'dir': [-1, 0, 0]},
            'right_jet_r':  {'x':  0.00870, 'y': -0.03848, 'z':  0.00350, 'dir': [-1, 0, 0]},
        }

        self.max_thrust = 0.10  # N (Peak PJA force)
        self.get_logger().info('PJA Allocation Controller Active (Persistent Wrench Mode).')

    def publish_map_to_odom_static(self):
        static_tf = TransformStamped()
        static_tf.header.stamp = self.get_clock().now().to_msg()
        static_tf.header.frame_id = 'map'
        static_tf.child_frame_id = 'odom'
        static_tf.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(static_tf)

    def imu_callback(self, msg: Imu):
        self.q = [msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z]
        self.current_gz = msg.angular_velocity.z  # rad/s around body Z

    def odom_callback(self, msg: Odometry):
        if not self.model_ready:                  
            self.model_ready = True
            self.get_logger().info('Model verified in Gazebo via /odom. Persistent wrench pipeline ready.')
        self.current_q = msg.pose.pose.orientation  
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
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
        if not self.model_ready:
            return

        dt_cmd = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if dt_cmd > self.cmd_timeout:
            self.cmd_linear = [0.0, 0.0, 0.0]
            self.cmd_angular = [0.0, 0.0, 0.0]

        u_surge, u_heave = self.cmd_linear[0], self.cmd_linear[2]
        u_pitch, u_yaw = self.cmd_angular[1], self.cmd_angular[2]

        # ACTIVE RATE DAMPING: Subtract Kd * omega_z to prevent runaway rotational integration
        Kd_yaw = 0.08
        damped_yaw = u_yaw - Kd_yaw * getattr(self, 'current_gz', 0.0)
        yaw_gain = 0.3 * damped_yaw

        surge_pitch_comp = 0.15 * abs(u_surge)

        # Continuous baseline demands (keeps all 4 horizontal jets active during turning)
        demands = {
            'hover_front': float(np.clip(u_heave + u_pitch + surge_pitch_comp, 0.0, 1.0)),
            'hover_rear':  float(np.clip(u_heave - u_pitch - surge_pitch_comp, 0.0, 1.0)),
            'left_jet_f':  float(np.clip(u_surge - yaw_gain, 0.0, 1.0)),
            'right_jet_f': float(np.clip(u_surge + yaw_gain, 0.0, 1.0)),
            'left_jet_r':  float(np.clip(-u_surge + yaw_gain, 0.0, 1.0)),  # Fixed sign
            'right_jet_r': float(np.clip(-u_surge - yaw_gain, 0.0, 1.0)),  # Fixed sign
        }
        # Check if any thruster is active
        if not any(d >= 1e-4 for d in demands.values()):
            return  # Instantaneous wrenches clear automatically when publishing stops!

        body_fx, body_fy, body_fz = 0.0, 0.0, 0.0
        body_tx, body_ty, body_tz = 0.0, 0.0, 0.0

        for name, demand in demands.items():
            if demand < 1e-4:
                continue

            thrust_mag = demand * self.max_thrust
            pja = self.pja_offsets[name]
            fx, fy, fz = pja['dir'][0] * thrust_mag, pja['dir'][1] * thrust_mag, pja['dir'][2] * thrust_mag
            rx, ry, rz = pja['x'], pja['y'], pja['z']

            body_fx += fx
            body_fy += fy
            body_fz += fz

            body_tx += ry * fz - rz * fy
            body_ty += rz * fx - rx * fz
            body_tz += rx * fy - ry * fx

        # Deadband small guidance chatter
        if abs(u_yaw) < 1e-3:
            body_tz = 0.0
        if abs(u_pitch) < 1e-3:
            body_ty = 0.0
            body_tx = 0.0

        # CRITICAL FIX: Rotate Body Frame Wrench -> World Frame Wrench
        # Gazebo's /wrench topic applies forces in the World coordinate system.
        q = getattr(self, 'current_q', None)
        if q is not None:
            w, x, y, z = q.w, q.x, q.y, q.z
            
            # Rotation Matrix derived from quaternion
            R11, R12, R13 = 1 - 2*(y**2 + z**2), 2*(x*y - w*z),     2*(x*z + w*y)
            R21, R22, R23 = 2*(x*y + w*z),       1 - 2*(x**2 + z**2), 2*(y*z - w*x)
            R31, R32, R33 = 2*(x*z - w*y),       2*(y*z + w*x),     1 - 2*(x**2 + y**2)

            world_fx = R11*body_fx + R12*body_fy + R13*body_fz
            world_fy = R21*body_fx + R22*body_fy + R23*body_fz
            world_fz = R31*body_fx + R32*body_fy + R33*body_fz

            world_tx = R11*body_tx + R12*body_ty + R13*body_tz
            world_ty = R21*body_tx + R22*body_ty + R23*body_tz
            world_tz = R31*body_tx + R32*body_ty + R33*body_tz
        else:
            # Fallback if no odom received yet
            world_fx, world_fy, world_fz = body_fx, body_fy, body_fz
            world_tx, world_ty, world_tz = body_tx, body_ty, body_tz

        # Construct Instantaneous EntityWrench Message
        wrench_msg = EntityWrench()
        wrench_msg.entity.name = 'Centroid_body'
        wrench_msg.entity.type = 2  # MODEL entity type

        # Publish the rotated WORLD frame forces and torques
        wrench_msg.wrench.force.x = float(world_fx)
        wrench_msg.wrench.force.y = float(world_fy)
        wrench_msg.wrench.force.z = float(world_fz)

        wrench_msg.wrench.torque.x = float(world_tx)
        wrench_msg.wrench.torque.y = float(world_ty)
        wrench_msg.wrench.torque.z = float(world_tz)

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