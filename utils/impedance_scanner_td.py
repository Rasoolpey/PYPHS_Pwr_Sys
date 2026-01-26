"""
Time-Domain Impedance Scanner (Robust System Identification)
Uses Band-Limited White Noise + Welch's Method for smooth Bode plots.
Includes automatic sampling rate adjustment and Coherence diagnostics.
"""
import numpy as np
import scipy.signal as signal
from scipy.integrate import solve_ivp
from utils.system_builder import PowerSystemBuilder
from utils.system_coordinator import PowerSystemCoordinator

class ImpedanceScannerTD:
    def __init__(self, json_file):
        self.builder = PowerSystemBuilder(json_file)
        self.builder.build_all_components()
        self.coordinator = PowerSystemCoordinator(self.builder)
        
        self.n_gen = len(self.builder.generators)
        self.states_per_machine = 13
        self.total_states = self.n_gen * self.states_per_machine
        
        # Default parameters
        self.fs = 200.0  
        self.T_settle = 5.0 
        self.f_end = 20.0 # Default fallback
        
        # Injection signal storage
        self._inj_time = None
        self._inj_signal_d = None
        self._inj_signal_q = None

    def list_buses(self):
        return [{'idx': i, 'name': f"Gen {i+1} (Bus {m['bus']})", 'bus_id': m['bus']} 
                for i, m in enumerate(self.builder.gen_metadata)]

    def _extract_param(self, core, param_name, default=1.0):
        for k, v in core.subs.items():
            if param_name in str(k):
                return float(v)
        return default

    def _solve_network_with_injection(self, gen_states, target_bus, I_inj_complex):
        # 1. Setup Matrices
        Ybus = self.coordinator.Ybus_base
        Xd2 = 0.25 * (100.0 / 900.0) 
        y_gen = 1.0 / (1j * Xd2)
        
        # 2. Internal Voltages
        E_sys = np.zeros(self.n_gen, dtype=complex)
        for i in range(self.n_gen):
            meta = self.builder.gen_metadata[i]
            delta, _, _, _, psi_f, psi_kd, psi_kq = gen_states[i]
            psi2d = meta['gd1'] * psi_f + meta['gd2'] * (meta['xd1'] - meta['xl']) * psi_kd
            psi2q = (1 - meta['gq1']) * psi_kq
            E_int = psi2d + 1j * psi2q
            E_sys[i] = (np.real(E_int) * np.cos(delta) - np.imag(E_int) * np.sin(delta)) + \
                       1j * (np.real(E_int) * np.sin(delta) + np.imag(E_int) * np.cos(delta))

        # 3. Iterative Solve
        I_gen_source = E_sys * y_gen
        S_load = self.coordinator.load_P + 1j * self.coordinator.load_Q
        V_term = np.ones(self.n_gen, dtype=complex)
        Y_total = Ybus + np.eye(self.n_gen) * y_gen
        
        for _ in range(3):
            I_load = np.conj(S_load / V_term)
            I_rhs = I_gen_source - I_load
            if target_bus >= 0:
                I_rhs[target_bus] += I_inj_complex
            V_term = np.linalg.solve(Y_total, I_rhs)
            
        # 4. Currents
        I_out = (E_sys - V_term) * y_gen
        
        Vd = np.zeros(self.n_gen)
        Vq = np.zeros(self.n_gen)
        Id = np.zeros(self.n_gen)
        Iq = np.zeros(self.n_gen)
        
        deltas = gen_states[:, 0]
        for i in range(self.n_gen):
            delta = deltas[i]
            v_r, v_i = np.real(V_term[i]), np.imag(V_term[i])
            i_r, i_i = np.real(I_out[i]), np.imag(I_out[i])
            Vd[i] = v_r * np.cos(delta) + v_i * np.sin(delta)
            Vq[i] = -v_r * np.sin(delta) + v_i * np.cos(delta)
            Id[i] = i_r * np.cos(delta) + i_i * np.sin(delta)
            Iq[i] = -i_r * np.sin(delta) + i_i * np.cos(delta)
            
        return Vd, Vq, Id, Iq

    def _get_injection_current_interp(self, t):
        if self._inj_time is None or t > self._inj_time[-1]: return 0j
        val_d = np.interp(t, self._inj_time, self._inj_signal_d)
        val_q = np.interp(t, self._inj_time, self._inj_signal_q)
        return val_d + 1j * val_q

    def _dynamics_with_injection(self, t, x_flat, target_bus):
        x = x_flat.reshape(self.n_gen, self.states_per_machine)
        gen_states = x[:, :7]
        
        I_inj_c = self._get_injection_current_interp(t) if target_bus >= 0 else 0j
        Vd, Vq, Id, Iq = self._solve_network_with_injection(gen_states, target_bus, I_inj_c)
        
        dxdt = np.zeros_like(x)
        for i in range(self.n_gen):
            meta = self.builder.gen_metadata[i]
            gen_core = self.builder.generators[i]
            exc_core = self.builder.exciters[i]
            gov_core = self.builder.governors[i]
            
            delta, p, psi_d, psi_q, psi_f, psi_kd, psi_kq = gen_states[i]
            vm, vr1, vr2, vf = x[i, 7:11]
            x1_gov, x2_gov = x[i, 11:13]
            
            omega = p / meta['M']
            gov_meta = self.builder.gov_metadata[i]
            Tm = (gov_meta['T2']/gov_meta['T3'])*(x1_gov - x2_gov) + x2_gov - gov_meta['Dt']*(omega-1.0)
            Te = Vd[i]*Id[i] + Vq[i]*Iq[i]
            
            dxdt[i, 0] = meta['omega_b'] * (omega - 1.0)
            dxdt[i, 1] = Tm - Te - meta['D']*(omega - 1.0)
            dxdt[i, 2] = Vd[i] - meta['ra']*Id[i] + meta['omega_b']*omega*psi_q
            dxdt[i, 3] = Vq[i] - meta['ra']*Iq[i] - meta['omega_b']*omega*psi_d
            
            Td10 = self._extract_param(gen_core, 'Td10', 8.0)
            Td20 = self._extract_param(gen_core, 'Td20', 0.03)
            Tq20 = self._extract_param(gen_core, 'Tq20', 0.05)
            
            Efd = vr1
            dxdt[i, 4] = (Efd - psi_f) / Td10
            dxdt[i, 5] = -psi_kd / Td20
            dxdt[i, 6] = -psi_kq / Tq20
            
            Vt = np.sqrt(Vd[i]**2 + Vq[i]**2)
            exc_meta = self.builder.exc_metadata[i]
            TR = self._extract_param(exc_core, 'TR', 0.02)
            TA = self._extract_param(exc_core, 'TA', 0.02)
            KA = self._extract_param(exc_core, 'KA', 200.0)
            Vref = self._extract_param(exc_core, 'Vref', 1.0)

            dxdt[i, 7] = (Vt - vm) / TR
            Verr = Vref - vm
            dxdt[i, 8] = (KA * Verr - vr1) / TA
            
            gate = gov_meta['Pref'] + (gov_meta['wref'] - omega)/gov_meta['R']
            dxdt[i, 11] = (gate - x1_gov) * 10.0 / gov_meta['T1']
            dxdt[i, 12] = (x1_gov - x2_gov) * 10.0 / gov_meta['T3']
            
        return dxdt.flatten()

    def generate_noise_signal(self, duration, f_max, amplitude):
        n_samples = int(duration * self.fs)
        t = np.linspace(0, duration, n_samples)
        
        noise_d = np.random.normal(0, 1, n_samples)
        noise_q = np.random.normal(0, 1, n_samples)
        
        sos = signal.butter(4, f_max, 'low', fs=self.fs, output='sos')
        filtered_d = signal.sosfilt(sos, noise_d)
        filtered_q = signal.sosfilt(sos, noise_q)
        
        # Normalize
        if np.std(filtered_d) > 0:
            filtered_d = filtered_d / np.std(filtered_d) * amplitude
        if np.std(filtered_q) > 0:
            filtered_q = filtered_q / np.std(filtered_q) * amplitude
        
        return t, filtered_d, filtered_q

    def run_scan(self, bus_idx, f_max=20.0, amplitude=0.01, duration=60.0):
        """Run Scan with White Noise Injection"""
        print(f"Starting System ID on Bus {bus_idx}...")
        self.f_end = f_max  # FIX: Save f_max for post-processing
        
        # Auto-adjust sampling
        required_fs = 10.0 * f_max
        if required_fs > self.fs:
            print(f"  Adjusting sampling rate from {self.fs} Hz to {required_fs} Hz")
            self.fs = required_fs
        
        print(f"  Generating {duration}s White Noise (0-{f_max} Hz)...")
        self._inj_time, self._inj_signal_d, self._inj_signal_q = \
            self.generate_noise_signal(duration, f_max, amplitude)
        
        print("  Settling system...")
        x0_flat = np.zeros(self.total_states)
        for i in range(self.n_gen):
            x0_flat[i*13 + 0] = 0.1 
            x0_flat[i*13 + 1] = self.builder.gen_metadata[i]['M']
            x0_flat[i*13 + 4] = 1.3
            x0_flat[i*13 + 7] = 1.0
            x0_flat[i*13 + 11] = 7.0
            x0_flat[i*13 + 12] = 7.0

        sol_settle = solve_ivp(
            lambda t, x: self._dynamics_with_injection(t, x, -1),
            (0, self.T_settle), x0_flat, method='RK45',
            max_step=1.0/self.fs 
        )
        x_start = sol_settle.y[:, -1]
        
        print(f"  Simulating response...")
        sol = solve_ivp(
            lambda t, x: self._dynamics_with_injection(t, x, bus_idx),
            (0, duration), x_start, t_eval=self._inj_time, method='RK45',
            max_step=1.0/self.fs
        )
        
        return sol, bus_idx

    def post_process_tfe(self, sol, bus_idx):
        """Calculate Impedance using TFE (Welch)"""
        print("  Processing Transfer Function Estimate...")
        t = sol.t
        
        V_complex = np.zeros(len(t), dtype=complex)
        I_complex = np.zeros(len(t), dtype=complex)
        
        for k in range(len(t)):
            I_inj = self._get_injection_current_interp(t[k])
            I_complex[k] = I_inj
            
            x_k = sol.y[:, k].reshape(self.n_gen, self.states_per_machine)
            gen_states = x_k[:, :7]
            
            Vd, Vq, _, _ = self._solve_network_with_injection(gen_states, bus_idx, I_inj)
            V_complex[k] = Vd[bus_idx] + 1j*Vq[bus_idx]
            
        # Diagnostics
        v_rms = np.std(V_complex)
        i_rms = np.std(I_complex)
        print(f"  > Signal Statistics: V_rms={v_rms:.6f}, I_rms={i_rms:.6f}")
        
        if i_rms < 1e-9:
            print("  ! WARNING: Injection current is effectively zero. Check amplitude.")
            
        V_ac = signal.detrend(V_complex, type='constant')
        I_ac = signal.detrend(I_complex, type='constant')
        
        # Calculate Spectra
        n_seg = min(4096, len(t))
        f, P_iv = signal.csd(I_ac, V_ac, fs=self.fs, nperseg=n_seg, return_onesided=False)
        f, P_ii = signal.welch(I_ac, fs=self.fs, nperseg=n_seg, return_onesided=False)
        f, P_vv = signal.welch(V_ac, fs=self.fs, nperseg=n_seg, return_onesided=False)
        
        # Sort
        f_shift = np.fft.fftshift(f)
        P_iv_shift = np.fft.fftshift(P_iv)
        P_ii_shift = np.fft.fftshift(P_ii)
        P_vv_shift = np.fft.fftshift(P_vv)
        
        pos_mask = f_shift > 0
        freqs = f_shift[pos_mask]
        P_iv = P_iv_shift[pos_mask]
        P_ii = P_ii_shift[pos_mask]
        P_vv = P_vv_shift[pos_mask]
        
        # Impedance
        P_ii[np.abs(P_ii) < 1e-15] = 1e-15
        Z_est = P_iv / P_ii
        
        # Coherence
        P_vv[np.abs(P_vv) < 1e-15] = 1e-15
        Coherence = (np.abs(P_iv)**2) / (P_ii * P_vv)
        
        # DEBUG: Print Average Coherence
        avg_coh = np.mean(Coherence)
        print(f"  > Average Coherence: {avg_coh:.4f}")
        
        # FILTER: Only filter by frequency range (Removed strict coherence filter)
        valid_mask = (freqs < self.f_end)
        
        return freqs[valid_mask], Z_est[valid_mask]