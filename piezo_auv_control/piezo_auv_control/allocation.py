
import numpy as np

# Single channel ordering used everywhere (CSV columns, /piezo_signals, pja_controller).
CHANNELS = (
    'hover_front',
    'hover_rear',
    'left_jet_f',
    'left_jet_r',
    'right_jet_f',
    'right_jet_r',
)

# Body-frame jet positions [m]; taken from Centroid_body.urdf.xacro.
POSITIONS = np.array([
    [0.03828, 0.00000, -0.00520],   # hover_front
    [-0.03828, 0.00000, -0.00520],  # hover_rear
    [-0.00870, 0.03848, 0.00350],   # left_jet_f
    [0.00870, 0.03848, 0.00350],    # left_jet_r
    [-0.00870, -0.03848, 0.00350],  # right_jet_f
    [0.00870, -0.03848, 0.00350],   # right_jet_r
])

# Thrust direction applied to the vehicle by each jet (body frame).
DIRECTIONS = np.array([
    [0.0, 0.0, 1.0],   # hover_front
    [0.0, 0.0, 1.0],   # hover_rear
    [1.0, 0.0, 0.0],   # left_jet_f
    [-1.0, 0.0, 0.0],  # left_jet_r
    [1.0, 0.0, 0.0],   # right_jet_f
    [-1.0, 0.0, 0.0],  # right_jet_r
])


def allocate(u_surge, u_heave, u_pitch, u_yaw, yaw_gain=0.8, pitch_comp=0.05):
    """Map normalised commands to per-jet demands in [0, 1] matching CHANNELS order."""
    yaw = yaw_gain * u_yaw
    comp = pitch_comp * abs(u_surge)
    demands = np.array([
        u_heave + u_pitch + comp,   # hover_front
        u_heave - u_pitch - comp,   # hover_rear
        u_surge - yaw,              # left_jet_f
        -u_surge + yaw,             # left_jet_r
        u_surge + yaw,              # right_jet_f
        -u_surge - yaw,             # right_jet_r
    ])
    return np.clip(demands, 0.0, 1.0)


def body_wrench(forces):
    """Net body-frame force (3,) and torque (3,) from per-jet forces (6,) [N]."""
    per_jet = np.asarray(forces, dtype=float)[:, None] * DIRECTIONS
    force = per_jet.sum(axis=0)
    torque = np.cross(POSITIONS, per_jet).sum(axis=0)
    return force, torque