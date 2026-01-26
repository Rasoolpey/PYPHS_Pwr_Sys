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

        # 3. Iterative Solve (match fault_sim_modular)
        I_gen_source = E_sys * y_gen
        S_load = self.coordinator.load_P + 1j * self.coordinator.load_Q
        V_term = np.ones(self.n_gen, dtype=complex)
        Y_total = Ybus + np.eye(self.n_gen) * y_gen
        
        # First iteration
        I_load = np.conj(S_load / V_term)
        I_rhs = I_gen_source - I_load
        if target_bus >= 0:
            I_rhs[target_bus] += I_inj_complex
        V_term = np.linalg.solve(Y_total, I_rhs)
        
        # Refine (2nd iteration with safety)
        V_safe = np.where(np.abs(V_term) > 0.5, V_term, np.ones(self.n_gen, dtype=complex))
        I_load = np.conj(S_load / V_safe)
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
            
            # CRITICAL: Add exciter saturation
            VRMAX = exc_meta.get('VRMAX', 5.2)
            VRMIN = exc_meta.get('VRMIN', -4.16)
            vr1_unlimited = (KA * Verr - vr1) / TA
            
            if vr1 >= VRMAX and vr1_unlimited > 0:
                dxdt[i, 8] = 0.0
            elif vr1 <= VRMIN and vr1_unlimited < 0:
                dxdt[i, 8] = 0.0
            else:
                dxdt[i, 8] = vr1_unlimited
            
            dxdt[i, 9] = 0.0
            dxdt[i, 10] = 0.0
            
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

    def _initialize_equilibrium(self):
        """Proper equilibrium initialization (same as fault_sim_modular)"""
        print("  Finding equilibrium...")
        
        P_targets = np.array([7.459, 7.0, 7.0, 7.0])
        delta_init = np.array([0.08, 0.07, 0.06, 0.07])
        psi_f_init = np.array([1.3, 1.3, 1.3, 1.3])
        
        for iteration in range(50):
            gen_states = np.zeros((self.n_gen, 7))
            for i in range(self.n_gen):
                M = self.builder.gen_metadata[i]['M']
                gen_states[i] = [delta_init[i], M * 1.0, 0.0, 0.0, psi_f_init[i], 0.0, 0.0]
            
            Vd, Vq, Id, Iq = self._solve_network_with_injection(gen_states, -1, 0j)
            P_calc = Vd * Id + Vq * Iq
            V_mag = np.sqrt(Vd**2 + Vq**2)
            
            max_error = 0
            for i in range(self.n_gen):
                P_error = P_targets[i] - P_calc[i]
                delta_init[i] += 0.02 * P_error
                
                V_error = 1.0 - V_mag[i]
                psi_f_init[i] += 0.1 * V_error
                psi_f_init[i] = np.clip(psi_f_init[i], 0.5, 2.0)
                
                max_error = max(max_error, abs(P_error))
            
            if max_error < 1e-4:
                break
        
        # Build full state
        x0 = np.zeros((self.n_gen, 13))
        for i in range(self.n_gen):
            meta = self.builder.gen_metadata[i]
            psi_q = (-Vd[i] - meta['ra'] * Id[i]) / meta['omega_b']
            psi_d = (Vq[i] + meta['ra'] * Iq[i]) / meta['omega_b']
            
            x0[i, 0] = delta_init[i]
            x0[i, 1] = meta['M'] * 1.0
            x0[i, 2] = psi_d
            x0[i, 3] = psi_q
            x0[i, 4] = psi_f_init[i]
            x0[i, 7] = V_mag[i]
            x0[i, 8] = psi_f_init[i]
            
            P_eq = P_calc[i]
            x0[i, 11] = P_eq
            x0[i, 12] = P_eq
            self.builder.gov_metadata[i]['Pref'] = P_eq
            
            # Update Vref
            exc_core = self.builder.exciters[i]
            for key in exc_core.subs.keys():
                if 'Vref' in str(key):
                    exc_core.subs[key] = V_mag[i]
                    break
        
        return x0.flatten()

    def run_scan(self, bus_idx, f_max=20.0, amplitude=0.01, duration=60.0):
        """Run Scan with White Noise Injection"""
        print(f"Starting System ID on Bus {bus_idx}...")
        self.f_end = f_max
        
        # Auto-adjust sampling
        required_fs = 10.0 * f_max
        if required_fs > self.fs:
            print(f"  Adjusting sampling rate from {self.fs} Hz to {required_fs} Hz")
            self.fs = required_fs
        
        print(f"  Generating {duration}s White Noise (0-{f_max} Hz)...")
        self._inj_time, self._inj_signal_d, self._inj_signal_q = \
            self.generate_noise_signal(duration, f_max, amplitude)
        
        x_start = self._initialize_equilibrium()
        
        print(f"  Simulating response (target bus: {bus_idx})...")
        print(f"    Total points: {len(self._inj_time):,}")
        
        # Progress tracking with visual bar
        last_print = [0]
        start_time = [__import__('time').time()]
        
        def progress_callback(t, x):
            current_time = __import__('time').time()
            if current_time - last_print[0] > 1.0:  # Update every 1 second
                progress_pct = 100 * t / duration
                elapsed = current_time - start_time[0]
                eta = (elapsed / t * duration - elapsed) if t > 0 else 0
                
                # Create visual progress bar
                bar_length = 40
                filled = int(bar_length * t / duration)
                bar = '█' * filled + '░' * (bar_length - filled)
                
                # Print on same line using carriage return
                print(f'\r    [{bar}] {progress_pct:5.1f}% | {t:6.1f}/{duration:.0f}s | ETA: {eta/60:4.1f} min', end='', flush=True)
                last_print[0] = current_time
            return 1
        progress_callback.terminal = False
        
        sol = solve_ivp(
            lambda t, x: self._dynamics_with_injection(t, x, bus_idx),
            (0, duration), x_start, t_eval=self._inj_time, method='RK45',
            max_step=1.0/self.fs, events=progress_callback
        )
        
        # Final progress bar update (show 100%)
        bar_length = 40
        bar = '█' * bar_length
        elapsed_total = __import__('time').time() - start_time[0]
        print(f'\r    [{bar}] 100.0% | {duration:6.1f}/{duration:.0f}s | Total: {elapsed_total/60:4.1f} min')
        print('    ✓ Simulation complete!')
        
        # Extract comprehensive signals
        self._extract_detailed_signals(sol, bus_idx)
        
        return sol, bus_idx
    
    def _extract_detailed_signals(self, sol, bus_idx):
        """Extract all system signals for analysis"""
        print("  Extracting detailed signals...")
        
        N = len(sol.t)
        self.signals = {
            't': sol.t,
            'bus_idx': bus_idx,
            # Generator electrical
            'Vd': np.zeros(N),
            'Vq': np.zeros(N),
            'Vt': np.zeros(N),
            'Id': np.zeros(N),
            'Iq': np.zeros(N),
            'Pe': np.zeros(N),
            # Generator mechanical
            'delta': np.zeros(N),
            'omega': np.zeros(N),
            'Pm': np.zeros(N),
            # Controls
            'Efd': np.zeros(N),
            'Vref': np.zeros(N),
            'Gate': np.zeros(N),
            # Injection
            'Id_inj': np.zeros(N),
            'Iq_inj': np.zeros(N)
        }
        
        for k in range(N):
            x_k = sol.y[:, k].reshape(self.n_gen, 13)
            gen_states = x_k[:, :7]
            
            I_inj = self._get_injection_current_interp(sol.t[k])
            Vd, Vq, Id_gen, Iq_gen = self._solve_network_with_injection(gen_states, bus_idx, I_inj)
            
            i = bus_idx
            self.signals['Vd'][k] = Vd[i]
            self.signals['Vq'][k] = Vq[i]
            self.signals['Vt'][k] = np.sqrt(Vd[i]**2 + Vq[i]**2)
            self.signals['Id'][k] = Id_gen[i]
            self.signals['Iq'][k] = Iq_gen[i]
            self.signals['Pe'][k] = Vd[i]*Id_gen[i] + Vq[i]*Iq_gen[i]
            
            self.signals['delta'][k] = gen_states[i, 0]
            M = self.builder.gen_metadata[i]['M']
            self.signals['omega'][k] = gen_states[i, 1] / M
            
            self.signals['Efd'][k] = x_k[i, 8]
            self.signals['Gate'][k] = x_k[i, 11]
            
            # Mechanical power from governor states
            gov_m = self.builder.gov_metadata[i]
            x1, x2 = x_k[i, 11:13]
            omega = self.signals['omega'][k]
            self.signals['Pm'][k] = (gov_m['T2']/gov_m['T3'])*(x1 - x2) + x2 - gov_m['Dt']*(omega-1.0)
            
            # Exciter reference
            exc_core = self.builder.exciters[i]
            self.signals['Vref'][k] = self._extract_param(exc_core, 'Vref', 1.0)
            
            # Injection currents
            self.signals['Id_inj'][k] = np.real(I_inj)
            self.signals['Iq_inj'][k] = np.imag(I_inj)
        
        print(f"    Captured {len(self.signals)} signal types over {N} points")
    
    def plot_system_response(self, output_file='outputs/system_response.png'):
        """Plot comprehensive system response to injection"""
        if not hasattr(self, 'signals'):
            print("No signals to plot. Run scan first.")
            return
        
        import matplotlib.pyplot as plt
        
        t = self.signals['t']
        bus_name = self.list_buses()[self.signals['bus_idx']]['name']
        
        fig, axes = plt.subplots(4, 2, figsize=(14, 12))
        fig.suptitle(f'System Response to White Noise Injection - {bus_name}', fontsize=14)
        
        # Row 1: Voltages
        axes[0,0].plot(t, self.signals['Vd'], 'b-', label='Vd')
        axes[0,0].plot(t, self.signals['Vq'], 'r-', label='Vq')
        axes[0,0].set_ylabel('Voltage (pu)')
        axes[0,0].set_title('DQ Voltages')
        axes[0,0].legend()
        axes[0,0].grid(True, alpha=0.3)
        
        axes[0,1].plot(t, self.signals['Vt'], 'g-', linewidth=1.5)
        axes[0,1].axhline(1.0, color='k', linestyle='--', alpha=0.3)
        axes[0,1].set_ylabel('Terminal Voltage (pu)')
        axes[0,1].set_title('Terminal Voltage Magnitude')
        axes[0,1].grid(True, alpha=0.3)
        
        # Row 2: Currents
        axes[1,0].plot(t, self.signals['Id'], 'b-', label='Id (gen)')
        axes[1,0].plot(t, self.signals['Iq'], 'r-', label='Iq (gen)')
        axes[1,0].set_ylabel('Current (pu)')
        axes[1,0].set_title('Generator DQ Currents')
        axes[1,0].legend()
        axes[1,0].grid(True, alpha=0.3)
        
        axes[1,1].plot(t, self.signals['Id_inj'], 'b-', alpha=0.7, label='Id inj')
        axes[1,1].plot(t, self.signals['Iq_inj'], 'r-', alpha=0.7, label='Iq inj')
        axes[1,1].set_ylabel('Injection (pu)')
        axes[1,1].set_title('Injected Current')
        axes[1,1].legend()
        axes[1,1].grid(True, alpha=0.3)
        
        # Row 3: Mechanical
        axes[2,0].plot(t, np.degrees(self.signals['delta']), 'b-')
        axes[2,0].set_ylabel('Rotor Angle (deg)')
        axes[2,0].set_title('Rotor Angle')
        axes[2,0].grid(True, alpha=0.3)
        
        axes[2,1].plot(t, self.signals['omega'], 'g-')
        axes[2,1].axhline(1.0, color='k', linestyle='--', alpha=0.3)
        axes[2,1].set_ylabel('Speed (pu)')
        axes[2,1].set_title('Rotor Speed')
        axes[2,1].grid(True, alpha=0.3)
        
        # Row 4: Controls
        axes[3,0].plot(t, self.signals['Efd'], 'r-', label='Efd (AVR out)')
        axes[3,0].axhline(self.signals['Vref'][0], color='k', linestyle='--', alpha=0.3, label='Vref')
        axes[3,0].set_ylabel('Field Voltage (pu)')
        axes[3,0].set_xlabel('Time (s)')
        axes[3,0].set_title('Exciter Output')
        axes[3,0].legend()
        axes[3,0].grid(True, alpha=0.3)
        
        axes[3,1].plot(t, self.signals['Gate'], 'b-', label='Gate (Gov out)')
        axes[3,1].plot(t, self.signals['Pm'], 'g-', alpha=0.7, label='Pm (mech power)')
        axes[3,1].plot(t, self.signals['Pe'], 'r-', alpha=0.7, label='Pe (elec power)')
        axes[3,1].set_ylabel('Power/Gate (pu)')
        axes[3,1].set_xlabel('Time (s)')
        axes[3,1].set_title('Governor & Power')
        axes[3,1].legend()
        axes[3,1].grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"  System response plot saved to: {output_file}")
        return fig

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