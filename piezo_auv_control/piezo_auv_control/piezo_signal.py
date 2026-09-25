"""Classical electromechanical drive model and microsecond waveform synthesizer for PJA ceramics."""
import numpy as np


class PiezoDriveModel:
    def __init__(self, v_max_vpp=155.0, v_min_vpp=1.0, f_res_wet_hz=125.0,
                 q_factor=5.0, exponent=1.85, peak_force_n=0.015, burst_duty=0.33,
                 use_classical_2nd_order=True):
        self.v_max_vpp = float(v_max_vpp)
        self.v_min_vpp = float(v_min_vpp)
        self.f_res_wet_hz = float(f_res_wet_hz)
        self.q_factor = float(q_factor)
        self.exponent = float(exponent)
        self.peak_force_n = float(peak_force_n)
        self.burst_duty = float(burst_duty)
        self.use_classical_2nd_order = bool(use_classical_2nd_order)

    def demand_to_vpp(self, demand):
        """Map normalized demand in [0, 1] to Peak-to-Peak Voltage Vpp."""
        d = np.clip(demand, 0.0, 1.0)
        return np.where(d > 1e-4, self.v_min_vpp + d * (self.v_max_vpp - self.v_min_vpp), 0.0)

    def resonance_gain(self, f_carrier_hz):
        """Compute resonance attenuation factor G_res using standard 2nd-order or linear approximation."""
        if f_carrier_hz <= 0.0:
            return 0.0

        if self.use_classical_2nd_order:
            # Classical double-sided mechanical transfer function: 1 / sqrt(1 + Q^2 * (f/f_res - f_res/f)^2)
            freq_ratio = f_carrier_hz / self.f_res_wet_hz
            detuning_sq = (freq_ratio - (1.0 / freq_ratio)) ** 2
            return 1.0 / np.sqrt(1.0 + (self.q_factor ** 2) * detuning_sq)
        else:
            # First-order linear symmetric detuning approximation
            detuning = abs(f_carrier_hz - self.f_res_wet_hz) / self.f_res_wet_hz
            return 1.0 / np.sqrt(1.0 + (2.0 * self.q_factor * detuning) ** 2)

    def average_force(self, vpp, f_carrier_hz):
        """Compute average thrust force [N] from Vpp and carrier frequency."""
        v_norm = np.clip(vpp / self.v_max_vpp, 0.0, 1.0)
        g_res = self.resonance_gain(f_carrier_hz)
        
        # Computes F_avg = F_peak * (V/V_ref)^gamma * G_res * D_burst
        return self.peak_force_n * (v_norm ** self.exponent) * g_res * self.burst_duty


def synth_waveform(t_vec, vpp, f_carrier_hz, f_burst_hz, burst_duty, phase_offset, ramp_cycles=1.0, bipolar=False):
    """Synthesize high-frequency burst-modulated square carrier waveform."""
    n_ch = len(vpp)
    n_samples = len(t_vec)
    wave = np.zeros((n_ch, n_samples), dtype=np.float32)

    if n_samples == 0:
        return wave

    t_burst = 1.0 / f_burst_hz
    t_pulse = burst_duty * t_burst
    t_car = 1.0 / f_carrier_hz

    for i in range(n_ch):
        v = vpp[i]
        if v < 1.0:
            continue

        t_shifted = t_vec + phase_offset[i] * t_burst
        phase_in_burst = t_shifted % t_burst
        burst_gate = phase_in_burst < t_pulse

        carrier_square = (t_shifted % t_car) < (0.5 * t_car)
        
        if bipolar:
            sig = np.where(carrier_square, v * 0.5, -v * 0.5)
        else:
            sig = np.where(carrier_square, v, 0.0)

        wave[i] = sig * burst_gate

    return wave