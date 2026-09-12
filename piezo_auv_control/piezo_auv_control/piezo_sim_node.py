#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import numpy as np

class PiezoJetAUVController:
    """Hydrodynamic force mapper and dynamic solver."""
    def __init__(self):
        self.rho = 998.2          
        self.f_res_wet = 250.0    
        
        self.m = 0.794            
        self.Iz = 0.0053          
        self.X_dot_u = 0.158      
        self.N_dot_r = 0.0012     
        self.Cd_x = 0.85          
        self.Area_cross = 0.0081  

        self.alpha = np.radians(100.0) 
        self.lx = 0.12            
        self.ly = 0.04            
        
        self.pulsing_mode_active = True
        self.burst_duty_ratio = 0.4 

    def compute_thrust_force(self, V_pp, f_exec):
        if V_pp < 1.0:
            return 0.0
        Q_factor = 5.0
        freq_response = 1.0 / np.sqrt(1.0 + Q_factor**2 * ((f_exec / self.f_res_wet) - (self.f_res_wet / f_exec))**2)
        return 1.25e-4 * (V_pp / 300.0)**1.85 * freq_response

    def control_mapping(self, u_cmd, r_cmd, w_cmd=0.0, V_max=300.0):
        if abs(u_cmd) < 0.01 and abs(r_cmd) < 0.01 and abs(w_cmd) < 0.01:
            return 0.0, 0.0, np.zeros(6)

        V_left = np.clip(u_cmd * V_max + r_cmd * (V_max * 0.5), 0.0, V_max)
        V_right = np.clip(u_cmd * V_max - r_cmd * (V_max * 0.5), 0.0, V_max)
        V_hover = np.clip(abs(w_cmd) * V_max, 0.0, V_max)

        V_commanded = np.array([V_left, V_left, V_right, V_right, V_hover, V_hover])

        F_left = self.compute_thrust_force(V_left, self.f_res_wet)
        F_right = self.compute_thrust_force(V_right, self.f_res_wet)

        if self.pulsing_mode_active:
            F_left *= self.burst_duty_ratio
            F_right *= self.burst_duty_ratio

        tau_x = (F_left + F_right) * np.cos(self.alpha / 2.0)
        tau_n = (-F_left * self.ly + F_right * self.ly) * np.cos(self.alpha / 2.0) + \
                (-F_left * self.lx + F_right * self.lx) * np.sin(self.alpha / 2.0)

        return tau_x, tau_n, V_commanded

    def update_dynamics(self, state, tau_x, tau_n, dt):
        u, v, r = state
        D_u = 0.5 * self.rho * self.Cd_x * self.Area_cross * u * np.abs(u)
        D_r = 0.05 * r * np.abs(r) 

        scale_gain = 15.0
        u_dot = ((tau_x * scale_gain) - D_u + (self.m + self.X_dot_u) * v * r) / (self.m + self.X_dot_u)
        r_dot = ((tau_n * scale_gain) - D_r) / (self.Iz + self.N_dot_r)

        u_new = u + u_dot * dt
        v_new = 0.0 
        r_new = r + r_dot * dt

        return np.array([u_new, v_new, r_new])


class PiezoSimulationROSNode(Node):
    def __init__(self):
        super().__init__('piezo_simulation_node')

        self.model = PiezoJetAUVController()

        self.f_carrier = 250.0       
        self.f_burst = 5.0           
        self.sample_rate = 30.0    
        self.dt = 1.0 / self.sample_rate

        self.time_counter = 0.0
        self.body_state = np.array([0.0, 0.0, 0.0]) 
        self.pose_2d = np.array([0.0, 0.0, 0.0])   
        
        self.u_cmd = 0.0
        self.r_cmd = 0.0
        self.w_cmd = 0.0

        self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pub_signals = self.create_publisher(Float64MultiArray, '/piezo_signals', qos_profile)
        self.pub_markers = self.create_publisher(MarkerArray, '/piezo_pulse_markers', 10)
        self.pub_path = self.create_publisher(Path, '/executed_path', 10)
        self.pub_joint_states = self.create_publisher(JointState, '/joint_states', 10)
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # Set Path Frame to odom
        self.path_msg = Path()
        self.path_msg.header.frame_id = "odom"

        self.publish_static_transforms()
        self.timer = self.create_timer(self.dt, self.timer_callback)

    def publish_static_transforms(self):
        # Anchor map -> odom
        static_tf = TransformStamped()
        static_tf.header.stamp = self.get_clock().now().to_msg()
        static_tf.header.frame_id = 'map'
        static_tf.child_frame_id = 'odom'
        static_tf.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(static_tf)

    def cmd_vel_callback(self, msg: Twist):
        self.u_cmd = np.clip(msg.linear.x, 0.0, 1.0)
        self.r_cmd = np.clip(msg.angular.z, -1.0, 1.0)
        self.w_cmd = np.clip(msg.linear.z, -1.0, 1.0)

    def generate_waveform(self, t, target_v):
        if target_v < 1.0:
            return 0.0
        t_burst = t % (1.0 / self.f_burst)
        is_burst_on = t_burst < (self.model.burst_duty_ratio / self.f_burst)
        if not is_burst_on:
            return 0.0
        t_carrier = t % (1.0 / self.f_carrier)
        return target_v if t_carrier < (0.5 / self.f_carrier) else 0.0

    def timer_callback(self):
        self.time_counter += self.dt
        t = self.time_counter
        now = self.get_clock().now().to_msg()

        # Update dynamic states
        tau_x, tau_n, V_cmd = self.model.control_mapping(self.u_cmd, self.r_cmd, self.w_cmd)
        self.body_state = self.model.update_dynamics(self.body_state, tau_x, tau_n, self.dt)
        u, v, r = self.body_state

        self.pose_2d[2] += r * self.dt
        self.pose_2d[0] += (u * np.cos(self.pose_2d[2]) - v * np.sin(self.pose_2d[2])) * self.dt
        self.pose_2d[1] += (u * np.sin(self.pose_2d[2]) + v * np.cos(self.pose_2d[2])) * self.dt

        # Publish Joint States
        js = JointState()
        js.header.stamp = now
        js.name = ['hover_front_joint', 'hover_rear_joint', 'left_jet_f_joint', 'left_jet_r_joint', 'right_jet_f_joint', 'right_jet_r_joint']
        js.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.pub_joint_states.publish(js)

        # Dynamic TF: odom -> base_footprint
        cy = np.cos(self.pose_2d[2] * 0.5)
        sy = np.sin(self.pose_2d[2] * 0.5)

        t_tf = TransformStamped()
        t_tf.header.stamp = now
        t_tf.header.frame_id = 'odom'
        t_tf.child_frame_id = 'base_footprint'
        t_tf.transform.translation.x = float(self.pose_2d[0])
        t_tf.transform.translation.y = float(self.pose_2d[1])
        t_tf.transform.translation.z = 0.0
        t_tf.transform.rotation.x = 0.0
        t_tf.transform.rotation.y = 0.0
        t_tf.transform.rotation.z = float(sy)
        t_tf.transform.rotation.w = float(cy)
        self.tf_broadcaster.sendTransform(t_tf)

        # Publish 6-Channel Waveforms
        v_inst = [self.generate_waveform(t, v_val) for v_val in V_cmd]
        sig_msg = Float64MultiArray()
        sig_msg.data = [t] + [float(val) for val in v_inst]
        self.pub_signals.publish(sig_msg)

        # Always update Path Header Stamp
        self.path_msg.header.stamp = now

        # Append Path & Markers when moving or command is active
        if abs(u) > 0.001 or abs(r) > 0.001 or max(V_cmd) > 1.0:
            pose = PoseStamped()
            pose.header.frame_id = "odom"
            pose.header.stamp = now
            pose.pose.position.x = float(self.pose_2d[0])
            pose.pose.position.y = float(self.pose_2d[1])
            pose.pose.position.z = 0.0
            pose.pose.orientation.z = float(sy)
            pose.pose.orientation.w = float(cy)
            
            self.path_msg.poses.append(pose)
            self.pub_path.publish(self.path_msg)

            if v_inst[0] > 0.0 or v_inst[2] > 0.0:
                self.publish_marker(self.pose_2d[0], self.pose_2d[1], max(V_cmd), now)

    def publish_marker(self, x, y, magnitude, stamp):
        markers = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = stamp
        marker.ns = "piezo_pulses"
        marker.id = int(self.time_counter * 100) % 5000
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = 0.0
        marker.scale.x = (magnitude / 300.0) * 0.04 + 0.01
        marker.scale.y = marker.scale.x
        marker.scale.z = marker.scale.x
        marker.color.a = 0.8
        marker.color.r = 0.0
        marker.color.g = 0.4
        marker.color.b = 1.0
        markers.markers.append(marker)
        self.pub_markers.publish(markers)

def main(args=None):
    rclpy.init(args=args)
    node = PiezoSimulationROSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()