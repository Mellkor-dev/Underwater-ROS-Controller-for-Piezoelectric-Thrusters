#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from ros_gz_interfaces.msg import Entity, EntityWrench
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import numpy as np

from piezo_auv_control.allocation import CHANNELS, POSITIONS, DIRECTIONS, allocate, body_wrench

class PJAControllerOriented(Node):
    def __init__(self):
        super().__init__('pja_controller_oriented')

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

        self.model_ready = False
        self.max_thrust = 0.05  # Peak PJA force (50 mN)
        self.get_logger().info('PJA Allocation Controller Active (Unified Allocation Pipeline).')

    def publish_map_to_odom_static(self):
        static_tf = TransformStamped()
        static_tf.header.stamp = self.get_clock().now().to_msg()
        static_tf.header.frame_id = 'map'
        static_tf.child_frame_id = 'odom'
        static_tf.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(static_tf)

    def imu_callback(self, msg: Imu):
        self.current_gz = msg.angular_velocity.z

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

    def control_loop(self):
        if not self.model_ready:
            return

        dt_cmd = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if dt_cmd > self.cmd_timeout:
            self.cmd_linear = [0.0, 0.0, 0.0]
            self.cmd_angular = [0.0, 0.0, 0.0]

        u_surge, u_heave = self.cmd_linear[0], self.cmd_linear[2]
        u_pitch, u_yaw = self.cmd_angular[1], self.cmd_angular[2]

        Kd_yaw = 0.02
        damped_yaw = u_yaw - Kd_yaw * getattr(self, 'current_gz', 0.0)

        # Execute shared allocation pipeline
        demands = allocate(
            u_surge=u_surge, 
            u_heave=u_heave, 
            u_pitch=u_pitch, 
            u_yaw=damped_yaw, 
            yaw_gain=0.8, 
            pitch_comp=0.05
        )

        forces = demands * self.max_thrust
        body_f, body_t = body_wrench(forces)

        # Deadband chatter
        if abs(u_yaw) < 1e-3:
            body_t[2] = 0.0
        if abs(u_pitch) < 1e-3:
            body_t[0] = 0.0
            body_t[1] = 0.0

        # Rotate Body Frame Wrench -> World Frame Wrench
        q = getattr(self, 'current_q', None)
        if q is not None and (q.w**2 + q.x**2 + q.y**2 + q.z**2) > 0.1:
            w, x, y, z = q.w, q.x, q.y, q.z
            
            R11, R12, R13 = 1 - 2*(y**2 + z**2), 2*(x*y - w*z),     2*(x*z + w*y)
            R21, R22, R23 = 2*(x*y + w*z),       1 - 2*(x**2 + z**2), 2*(y*z - w*x)
            R31, R32, R33 = 2*(x*z - w*y),       2*(y*z + w*x),     1 - 2*(x**2 + y**2)

            R = np.array([[R11, R12, R13], [R21, R22, R23], [R31, R32, R33]])
            world_f = R @ body_f
            world_t = R @ body_t
        else:
            world_f = body_f
            world_t = body_t

        # Publish rotated WORLD frame wrench to Gazebo
        wrench_msg = EntityWrench()
        wrench_msg.entity.name = 'Centroid_body'
        wrench_msg.entity.type = 2  # MODEL entity type

        wrench_msg.wrench.force.x = float(world_f[0])
        wrench_msg.wrench.force.y = float(world_f[1])
        wrench_msg.wrench.force.z = float(world_f[2])

        wrench_msg.wrench.torque.x = float(world_t[0])
        wrench_msg.wrench.torque.y = float(world_t[1])
        wrench_msg.wrench.torque.z = float(world_t[2])

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