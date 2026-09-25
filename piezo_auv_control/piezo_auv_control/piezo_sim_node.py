#!/usr/bin/env python3
import csv
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from visualization_msgs.msg import Marker, MarkerArray

from piezo_auv_control.allocation import CHANNELS, allocate
from piezo_auv_control.piezo_signal import PiezoDriveModel, synth_waveform

class PiezoSignalGenerator(Node):
    def __init__(self):
        super().__init__('piezo_signal_generator')

        params = {
            'control_rate_hz': 30.0,
            'waveform_fs_hz': 20000.0,
            'f_carrier_hz': 125.0,
            'f_res_wet_hz': 125.0,
            'q_factor': 5.0,
            'thrust_exponent': 1.85,
            'f_burst_hz': 5.0,
            'burst_duty': 0.33,
            'v_max_vpp': 155.0,
            'v_min_vpp': 1.0,
            'peak_force_n': 0.015,
            'stagger_left_right': True,
            'ramp_cycles': 1.0,
            'bipolar': False,
            'publish_waveform': True,
            'save_waveform': True,
            'waveform_max_seconds': 120.0,
            'log_path': '~/auv_ws/piezo_signals_log.csv',
            'cmd_timeout_s': 0.5,
            'yaw_gain': 0.8,
            'pitch_comp': 0.05,
        }
        for name, default in params.items():
            self.declare_parameter(name, default)
        self.p = {name: self.get_parameter(name).value for name in params}
        p = self.p

        self.dt_nominal = 1.0 / p['control_rate_hz']
        self.fs = float(p['waveform_fs_hz'])

        self.model = PiezoDriveModel(
            v_max_vpp=p['v_max_vpp'], v_min_vpp=p['v_min_vpp'],
            f_res_wet_hz=p['f_res_wet_hz'], q_factor=p['q_factor'],
            exponent=p['thrust_exponent'], peak_force_n=p['peak_force_n'],
            burst_duty=p['burst_duty']
        )

        self.phase_offset = np.zeros(len(CHANNELS))
        if p['stagger_left_right']:
            for i, name in enumerate(CHANNELS):
                if name.startswith('right'):
                    self.phase_offset[i] = 0.5

        self.cmd = np.zeros(4)
        self.yaw_rate = 0.0
        self.current_pos = np.zeros(3)
        
        self.last_cmd_time = None
        self.t_start = self.get_clock().now().nanoseconds * 1e-9
        self.last_tick = self.t_start
        self.sample_index = 0
        self.wave_chunks = []
        self.wave_samples_saved = 0

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pub_signals = self.create_publisher(Float64MultiArray, '/piezo_signals', qos)
        self.pub_wave = self.create_publisher(Float64MultiArray, '/piezo_waveform', qos)
        self.pub_markers = self.create_publisher(MarkerArray, '/piezo_pulse_markers', 10)

        self.csv_path = os.path.expanduser(p['log_path'])
        os.makedirs(os.path.dirname(self.csv_path) or '.', exist_ok=True)
        self.csv_file = open(self.csv_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        self.csv_writer.writerow(
            ['time_s'] + [f'v_{c}' for c in CHANNELS] +
            ['u_cmd', 'w_cmd', 'pitch_cmd', 'r_cmd', 'f_carrier_hz', 'burst_duty']
        )
        self.last_flush = time.monotonic()
        self.timer = self.create_timer(self.dt_nominal, self.timer_callback)

        self.get_logger().info('Piezo Signal Generator Active. Unified allocation pipeline synchronized to /odom.')

    def cmd_vel_callback(self, msg: Twist):
        self.cmd = np.clip([msg.linear.x, msg.linear.z, msg.angular.y, msg.angular.z], -1.0, 1.0)
        self.last_cmd_time = self.get_clock().now().nanoseconds * 1e-9

    def odom_callback(self, msg: Odometry):
        self.current_pos[0] = msg.pose.pose.position.x
        self.current_pos[1] = msg.pose.pose.position.y
        self.current_pos[2] = msg.pose.pose.position.z
        
        # Save orientation quaternion for world-frame vector transformations
        q = msg.pose.pose.orientation
        self.current_quat = np.array([q.x, q.y, q.z, q.w])
        self.yaw_rate = msg.twist.twist.angular.z

    def synthesize_chunk(self, now, vpp):
        target = int(round((now - self.t_start) * self.fs))
        nominal = int(round(self.fs * self.dt_nominal))
        n = target - self.sample_index
        
        if n <= 0:
            return None, None
        if n > 3 * nominal:
            self.sample_index = target - nominal
            n = nominal
            
        start = self.sample_index
        t_vec = (start + np.arange(n)) / self.fs
        
        wave = synth_waveform(
            t_vec, vpp, self.p['f_carrier_hz'], self.p['f_burst_hz'],
            self.p['burst_duty'], self.phase_offset, self.p['ramp_cycles'],
            self.p['bipolar']
        )
        self.sample_index = start + n
        return start, wave

    def timer_callback(self):
        p = self.p
        now_ros = self.get_clock().now()
        now = now_ros.nanoseconds * 1e-9
        self.last_tick = now
        t = now - self.t_start

        if self.last_cmd_time is None or now - self.last_cmd_time > p['cmd_timeout_s']:
            self.cmd = np.zeros(4)
            
        u_surge, u_heave, u_pitch, u_yaw = self.cmd

        Kd_yaw = 0.02
        damped_yaw = u_yaw - Kd_yaw * self.yaw_rate

        # Shared allocation call
        demands = allocate(
            u_surge=u_surge, 
            u_heave=u_heave, 
            u_pitch=u_pitch, 
            u_yaw=damped_yaw, 
            yaw_gain=p['yaw_gain'], 
            pitch_comp=p['pitch_comp']
        )
        vpp = np.array([self.model.demand_to_vpp(d) for d in demands])

        # Control envelope logging
        env_msg = Float64MultiArray()
        env_msg.data = [t] + [float(x) for x in vpp]
        self.pub_signals.publish(env_msg)
        
        self.csv_writer.writerow(
            [f'{t:.6f}'] + [f'{x:.3f}' for x in vpp] +
            [f'{u_surge:.5f}', f'{u_heave:.5f}', f'{u_pitch:.5f}', f'{u_yaw:.5f}',
             p['f_carrier_hz'], p['burst_duty']]
        )
        
        if time.monotonic() - self.last_flush > 1.0:
            self.csv_file.flush()
            self.last_flush = time.monotonic()

        # High-Rate Waveform Synthesis
        if p['publish_waveform'] or p['save_waveform']:
            start, wave = self.synthesize_chunk(now, vpp)
            if wave is not None:
                if p['publish_waveform']:
                    self.publish_waveform(start / self.fs, wave)
                if p['save_waveform'] and self.wave_samples_saved < p['waveform_max_seconds'] * self.fs:
                    self.wave_chunks.append((start, wave.astype(np.float32)))
                    self.wave_samples_saved += wave.shape[1]

        self.publish_marker(vpp, now_ros.to_msg())
        
    def publish_waveform(self, t0, wave):
        n_ch, n = wave.shape
        msg = Float64MultiArray()
        msg.layout.dim = [
            MultiArrayDimension(label='channel', size=n_ch, stride=n_ch * n),
            MultiArrayDimension(label='sample', size=n, stride=n),
        ]
        msg.layout.data_offset = 2
        msg.data = [float(t0), self.fs] + wave.ravel().tolist()
        self.pub_wave.publish(msg)

    def publish_marker(self, vpp_array, stamp):
        """Publish crisp, pulsed directional jet arrows attached to PJA nozzles."""
        markers = MarkerArray()
        
        # Exact PJA nozzle specs in base_link frame: (dx, dy, dz, jet_dir_x, jet_dir_y, jet_dir_z)
        # Note: Directions represent ejected fluid flow vectors pointing OUTWARD from the body.
        nozzle_specs = [
            ( 0.045,  0.0,   -0.015,  0.0,  0.0, -1.0),  # 0: Hover Front -> Ejects Down (-Z)
            (-0.045,  0.0,   -0.015,  0.0,  0.0, -1.0),  # 1: Hover Rear  -> Ejects Down (-Z)
            ( 0.03,   0.045,  0.0,    0.0,  1.0,  0.0),  # 2: Left Front  -> Ejects Left (+Y outward)
            (-0.03,   0.045,  0.0,    0.0,  1.0,  0.0),  # 3: Left Rear   -> Ejects Left (+Y outward)
            ( 0.03,  -0.045,  0.0,    0.0, -1.0,  0.0),  # 4: Right Front -> Ejects Right (-Y outward)
            (-0.03,  -0.045,  0.0,    0.0, -1.0,  0.0),  # 5: Right Rear  -> Ejects Right (-Y outward)
        ]

        # Extract current body orientation quaternion (qx, qy, qz, qw)
        qx, qy, qz, qw = self.current_quat
        
        # Body-to-World Rotation Matrix
        R = np.array([
            [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)]
        ])

        # Evaluate current time within the 5 Hz burst cycle to determine active pulsing
        t_now = self.get_clock().now().nanoseconds * 1e-9
        t_burst = 1.0 / self.p['f_burst_hz']          # 0.200 s (200 ms)
        t_pulse = self.p['burst_duty'] * t_burst      # 0.0667 s (66.7 ms active)

        for i, (dx, dy, dz, dir_x, dir_y, dir_z) in enumerate(nozzle_specs):
            v = vpp_array[i] if i < len(vpp_array) else 0.0
            
            # Calculate channel phase shift
            phase_shift = self.phase_offset[i] * t_burst
            phase_in_burst = (t_now + phase_shift) % t_burst
            is_active_burst = phase_in_burst < t_pulse

            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = stamp
            m.ns = 'pja_jets'
            m.id = i  # FIXED ID (0 to 5) so markers overwrite rather than stack infinitely

            # If voltage is below threshold OR we are currently in the OFF phase of the burst cycle
            if v < 10.0 or not is_active_burst:
                m.action = Marker.DELETE  # Instantly remove marker when jet is off
                markers.markers.append(m)
                continue

            # 1. Transform nozzle offset to world position
            body_offset = np.array([dx, dy, dz])
            world_nozzle_pos = self.current_pos + R.dot(body_offset)

            # 2. Transform jet flow direction vector to world orientation
            body_dir = np.array([dir_x, dir_y, dir_z])
            world_dir = R.dot(body_dir)

            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.lifetime.sec = 0  # Persists cleanly until updated/deleted on next cycle

            # Set arrow start (nozzle exit) and end (fluid tip)
            thrust_ratio = float(v / self.p['v_max_vpp'])
            arrow_length = 0.015 + 0.035 * thrust_ratio  # 1.5 cm to 5.0 cm length
            
            p_start = Marker().pose.position
            p_start.x, p_start.y, p_start.z = world_nozzle_pos
            
            p_end = Marker().pose.position
            p_end.x = world_nozzle_pos[0] + world_dir[0] * arrow_length
            p_end.y = world_nozzle_pos[1] + world_dir[1] * arrow_length
            p_end.z = world_nozzle_pos[2] + world_dir[2] * arrow_length

            m.points = [p_start, p_end]

            # Geometry Dimensions
            m.scale.x = 0.004  # Shaft diameter (4 mm)
            m.scale.y = 0.008  # Head diameter (8 mm)
            m.scale.z = 0.008  # Head length

            # Bright Cyan color during active firing pulse
            m.color.r = 0.0
            m.color.g = 0.9
            m.color.b = 1.0
            m.color.a = 0.95

            markers.markers.append(m)

        self.pub_markers.publish(markers)

    def save_waveform_file(self):
        if not self.wave_chunks:
            return
        first = self.wave_chunks[0][0]
        last = max(s + w.shape[1] for s, w in self.wave_chunks)
        out = np.zeros((len(CHANNELS), last - first), dtype=np.float32)
        for start, w in self.wave_chunks:
            out[:, start - first:start - first + w.shape[1]] = w
            
        path = os.path.splitext(self.csv_path)[0] + '_waveform.npz'
        np.savez_compressed(path, wave=out, fs=self.fs, t0=first / self.fs, channels=np.array(CHANNELS))
        self.get_logger().info(f'Saved 20kHz waveform ({out.shape[1] / self.fs:.1f} s) to {path}')

    def destroy_node(self):
        if hasattr(self, 'csv_file') and not self.csv_file.closed:
            self.csv_file.close()
        if self.p.get('save_waveform'):
            self.save_waveform_file()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PiezoSignalGenerator()
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