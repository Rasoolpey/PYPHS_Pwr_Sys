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

    def _solve_network_with_injection(self, gen_states, target_bus, I_inj_dq):
        """Network solver with dq-frame current injection.

        Args:
            gen_states: Generator states (n_gen x 7)
            target_bus: Bus index for injection (-1 for no injection)
            I_inj_dq: Complex injection current in DQ FRAME (Id + j*Iq)
                      This will be transformed to network (RI) frame using delta.

        Returns:
            Vd, Vq, Id, Iq: DQ frame voltages and currents
            V_term: Network frame (RI) terminal voltages
        """
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

        # 3. Transform injection from DQ to RI (network) frame
        I_inj_net = 0j
        if target_bus >= 0 and I_inj_dq != 0j:
            delta_target = gen_states[target_bus, 0]
            Id_inj = np.real(I_inj_dq)
            Iq_inj = np.imag(I_inj_dq)
            # Park transformation: dq -> RI (same as other scanners)
            I_inj_net = (Id_inj * np.cos(delta_target) - Iq_inj * np.sin(delta_target)) + \
                        1j * (Id_inj * np.sin(delta_target) + Iq_inj * np.cos(delta_target))

        # 4. Iterative Solve (match fault_sim_modular)
        I_gen_source = E_sys * y_gen
        S_load = self.coordinator.load_P + 1j * self.coordinator.load_Q
        V_term = np.ones(self.n_gen, dtype=complex)
        Y_total = Ybus + np.eye(self.n_gen) * y_gen

        # First iteration
        I_load = np.conj(S_load / V_term)
        I_rhs = I_gen_source - I_load
        if target_bus >= 0:
            I_rhs[target_bus] += I_inj_net
        V_term = np.linalg.solve(Y_total, I_rhs)

        # Refine (2nd iteration with safety)
        V_safe = np.where(np.abs(V_term) > 0.5, V_term, np.ones(self.n_gen, dtype=complex))
        I_load = np.conj(S_load / V_safe)
        I_rhs = I_gen_source - I_load
        if target_bus >= 0:
            I_rhs[target_bus] += I_inj_net
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

        # Return V_term (RI frame) along with dq quantities for impedance measurement
        return Vd, Vq, Id, Iq, V_term

    def _get_injection_current_interp(self, t):
        if self._inj_time is None or t > self._inj_time[-1]: return 0j
        val_d = np.interp(t, self._inj_time, self._inj_signal_d)
        val_q = np.interp(t, self._inj_time, self._inj_signal_q)
        return val_d + 1j * val_q

    def _dynamics_with_injection(self, t, x_flat, target_bus):
        x = x_flat.reshape(self.n_gen, self.states_per_machine)
        gen_states = x[:, :7]
        
        I_inj_c = self._get_injection_current_interp(t) if target_bus >= 0 else 0j
        Vd, Vq, Id, Iq, _ = self._solve_network_with_injection(gen_states, target_bus, I_inj_c)
        
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
        """Generate band-limited white noise for injection (dual-axis)."""
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
            
            Vd, Vq, Id, Iq, _ = self._solve_network_with_injection(gen_states, -1, 0j)
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
        
        # Print equilibrium diagnostics
        print(f"  Equilibrium voltages: {V_mag}")
        print(f"  Equilibrium powers: {P_calc}")
        
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
            
            # Update Vref to match equilibrium voltage
            # This prevents AVR from fighting the equilibrium
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
            # Generator electrical (dq frame)
            'Vd': np.zeros(N),
            'Vq': np.zeros(N),
            'Vt': np.zeros(N),
            'Id': np.zeros(N),
            'Iq': np.zeros(N),
            'Pe': np.zeros(N),
            # Terminal voltage in RI frame (for impedance measurement)
            'V_real': np.zeros(N),
            'V_imag': np.zeros(N),
            # Generator mechanical
            'delta': np.zeros(N),
            'omega': np.zeros(N),
            'Pm': np.zeros(N),
            # Controls
            'Efd': np.zeros(N),
            'Vref': np.zeros(N),
            'Gate': np.zeros(N),
            # Injection (RI frame)
            'Id_inj': np.zeros(N),
            'Iq_inj': np.zeros(N)
        }
        
        for k in range(N):
            x_k = sol.y[:, k].reshape(self.n_gen, 13)
            gen_states = x_k[:, :7]
            
            I_inj = self._get_injection_current_interp(sol.t[k])
            Vd, Vq, Id_gen, Iq_gen, V_term = self._solve_network_with_injection(gen_states, bus_idx, I_inj)
            
            i = bus_idx
            self.signals['Vd'][k] = Vd[i]
            self.signals['Vq'][k] = Vq[i]
            self.signals['Vt'][k] = np.sqrt(Vd[i]**2 + Vq[i]**2)
            self.signals['Id'][k] = Id_gen[i]
            self.signals['Iq'][k] = Iq_gen[i]
            self.signals['Pe'][k] = Vd[i]*Id_gen[i] + Vq[i]*Iq_gen[i]
            # Store RI frame voltage for impedance diagnostics
            self.signals['V_real'][k] = np.real(V_term[i])
            self.signals['V_imag'][k] = np.imag(V_term[i])
            
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
        """Calculate DQ-frame Impedance using TFE (Welch)

        We measure Z_dq = V_dq / I_dq where both are in the machine's dq frame.
        This matches the approach used by impedance_scanner.py and imtb_scanner.py.

        The injection current (I_d + j*I_q) is in dq frame, transformed to RI frame
        for network solution, then voltage is measured back in dq frame.
        """
        print("  Processing Transfer Function Estimate (DQ Frame)...")
        t = sol.t

        V_dq_complex = np.zeros(len(t), dtype=complex)
        I_inj_dq = np.zeros(len(t), dtype=complex)

        for k in range(len(t)):
            I_inj = self._get_injection_current_interp(t[k])
            I_inj_dq[k] = I_inj  # Injection in DQ frame

            x_k = sol.y[:, k].reshape(self.n_gen, self.states_per_machine)
            gen_states = x_k[:, :7]

            # Injection is transformed to RI inside _solve_network_with_injection
            # Output Vd, Vq are in DQ frame (matching injection frame)
            Vd, Vq, Id_gen, Iq_gen, V_term = self._solve_network_with_injection(gen_states, bus_idx, I_inj)
            V_dq_complex[k] = Vd[bus_idx] + 1j * Vq[bus_idx]  # DQ frame voltage

        # Diagnostics
        v_rms = np.std(V_dq_complex)
        i_inj_rms = np.std(I_inj_dq)
        v_mean = np.mean(np.abs(V_dq_complex))
        print(f"  > Signal Statistics (DQ Frame):")
        print(f"      V_dq_rms={v_rms:.6f} pu, V_dq_mean={v_mean:.6f} pu")
        print(f"      I_inj_rms={i_inj_rms:.6f} pu (injection current in dq)")
        print(f"      Quick Z_dq estimate: V/I={v_rms/i_inj_rms:.6f} pu")

        if i_inj_rms < 1e-9:
            print("  ! WARNING: Injection current is effectively zero.")

        if v_rms < 1e-4:
            print(f"  ! WARNING: Voltage response is very small ({v_rms:.6f} pu)!")

        # Detrend signals (remove DC component)
        V_ac = signal.detrend(V_dq_complex, type='constant')
        I_ac = signal.detrend(I_inj_dq, type='constant')

        # Calculate Spectra
        n_seg = min(4096, len(t))
        f, P_iv = signal.csd(I_ac, V_ac, fs=self.fs, nperseg=n_seg, return_onesided=False)
        f, P_ii = signal.welch(I_ac, fs=self.fs, nperseg=n_seg, return_onesided=False)
        f, P_vv = signal.welch(V_ac, fs=self.fs, nperseg=n_seg, return_onesided=False)

        # Sort frequencies
        f_shift = np.fft.fftshift(f)
        P_iv_shift = np.fft.fftshift(P_iv)
        P_ii_shift = np.fft.fftshift(P_ii)
        P_vv_shift = np.fft.fftshift(P_vv)

        pos_mask = f_shift > 0
        freqs = f_shift[pos_mask]
        P_iv = P_iv_shift[pos_mask]
        P_ii = P_ii_shift[pos_mask]
        P_vv = P_vv_shift[pos_mask]

        # Impedance Z = V/I
        P_ii[np.abs(P_ii) < 1e-15] = 1e-15
        Z_est = P_iv / P_ii

        # Coherence
        P_vv[np.abs(P_vv) < 1e-15] = 1e-15
        Coherence = (np.abs(P_iv)**2) / (P_ii * P_vv)

        avg_coh = np.mean(Coherence)
        print(f"  > Average Coherence: {avg_coh:.4f}")

        # Filter by frequency range
        valid_mask = (freqs < self.f_end)

        return freqs[valid_mask], Z_est[valid_mask]

    def compute_network_impedance(self, bus_idx, freqs):
        """Compute network impedance EXCLUDING the local generator at bus_idx.

        This gives the impedance looking INTO the grid from the bus,
        as if the local generator were disconnected.

        For a purely algebraic network (no dynamics), the impedance is
        frequency-independent and equals Z_network = inv(Y_bus)[bus_idx, bus_idx]
        where Y_bus includes all OTHER generators' admittances but NOT the target.
        """
        print(f"  Computing Network Impedance (excluding Gen {bus_idx+1})...")

        # Build Y_total but exclude the target generator's admittance
        Ybus = self.coordinator.Ybus_base.copy()
        Xd2 = 0.25 * (100.0 / 900.0)  # Xd'' on system base
        y_gen = 1.0 / (1j * Xd2)

        # Add all generators EXCEPT the target bus
        Y_network = Ybus.copy()
        for i in range(self.n_gen):
            if i != bus_idx:
                Y_network[i, i] += y_gen

        # Network impedance at target bus (self-impedance from Z = inv(Y))
        Z_network_matrix = np.linalg.inv(Y_network)
        Z_network_self = Z_network_matrix[bus_idx, bus_idx]

        # Network impedance is frequency-independent (purely algebraic)
        # Return same value for all frequencies
        Z_network = np.full(len(freqs), Z_network_self, dtype=complex)

        print(f"      Z_network = {np.abs(Z_network_self):.4f} pu @ {np.degrees(np.angle(Z_network_self)):.1f}°")

        return Z_network

    def compute_generator_impedance(self, bus_idx, freqs):
        """Compute synchronous generator impedance vs frequency.

        The generator impedance is the operational impedance of the synchronous
        machine. For GENROU model, the d-axis operational impedance is:

        Zd(s) = Ra + Xd''*s * (1 + s*Td'') * (1 + s*Td') / ((1 + s*Td0') * (1 + s*Td0''))

        At different frequencies:
        - f → 0: Z → Ra (stator resistance)
        - f = 60 Hz: Z ≈ j*Xd'' (subtransient reactance)
        - f → ∞: Z → Ra + j*ω*Ll (leakage)

        Note: Values are on SYSTEM BASE (100 MVA), not machine base.
        """
        print(f"  Computing Generator Impedance for Gen {bus_idx+1}...")

        meta = self.builder.gen_metadata[bus_idx]

        # Machine parameters (on machine base 900 MVA)
        Ra_m = meta['ra']       # Stator resistance
        Xd_m = meta.get('xd', 1.8)    # Synchronous reactance
        Xd1_m = meta['xd1']     # Transient reactance (Xd')
        Xd2_m = meta.get('xd2', 0.25)  # Subtransient reactance (Xd'')
        Xl_m = meta['xl']       # Leakage reactance

        # Time constants
        Td10 = 8.0   # d-axis transient open-circuit time constant
        Td20 = 0.03  # d-axis subtransient open-circuit time constant

        # Derived time constants (short-circuit)
        Td1 = Td10 * Xd1_m / Xd_m    # Td'
        Td2 = Td20 * Xd2_m / Xd1_m   # Td''

        # Convert to system base (100 MVA)
        base_ratio = 100.0 / 900.0
        Ra = Ra_m * base_ratio
        Xd = Xd_m * base_ratio
        Xd1 = Xd1_m * base_ratio
        Xd2 = Xd2_m * base_ratio
        Xl = Xl_m * base_ratio

        print(f"      Ra={Ra:.4f}, Xd={Xd:.4f}, Xd'={Xd1:.4f}, Xd''={Xd2:.4f} pu (system base)")
        print(f"      Td0'={Td10:.2f}s, Td0''={Td20:.3f}s, Td'={Td1:.4f}s, Td''={Td2:.6f}s")

        # Compute frequency-dependent impedance
        omega_b = 2 * np.pi * 60.0  # Base angular frequency (rad/s)
        Z_gen = np.zeros(len(freqs), dtype=complex)

        for i, f in enumerate(freqs):
            if f < 1e-6:
                # DC limit: just stator resistance
                Z_gen[i] = Ra + 1e-10j
            else:
                s = 1j * 2 * np.pi * f  # Laplace variable

                # Operational d-axis impedance (simplified GENROU model)
                # Zd(s) = Ra + Xd(s) where Xd(s) is the operational reactance
                # Xd(s) = Xd * (1 + s*Td') * (1 + s*Td'') / ((1 + s*Td0') * (1 + s*Td0''))

                num = (1 + s * Td1) * (1 + s * Td2)
                den = (1 + s * Td10) * (1 + s * Td20)
                Xd_op = Xd * num / den

                # Convert reactance to impedance (multiply by j for inductance)
                Z_gen[i] = Ra + 1j * Xd_op

        # Print some reference values
        idx_1hz = np.argmin(np.abs(freqs - 1.0))
        idx_10hz = np.argmin(np.abs(freqs - 10.0))
        idx_50hz = np.argmin(np.abs(freqs - 50.0))

        if len(freqs) > 0:
            print(f"      Z_gen @ 1 Hz: {np.abs(Z_gen[idx_1hz]):.4f} pu")
            print(f"      Z_gen @ 10 Hz: {np.abs(Z_gen[idx_10hz]):.4f} pu")
            print(f"      Z_gen @ 50 Hz: {np.abs(Z_gen[idx_50hz]):.4f} pu")

        return Z_gen

    def compute_all_impedances(self, sol, bus_idx):
        """Compute all three impedance types and return them.

        Returns:
            freqs: Frequency array
            Z_total: Full Thevenin impedance (measured from simulation)
            Z_network: Network impedance (excluding local generator)
            Z_gen: Generator impedance (analytical)
        """
        print("\n" + "="*60)
        print("COMPUTING ALL IMPEDANCE COMPONENTS")
        print("="*60)

        # 1. Full Thevenin impedance (from simulation)
        freqs, Z_total = self.post_process_tfe(sol, bus_idx)

        # 2. Network impedance (analytical)
        Z_network = self.compute_network_impedance(bus_idx, freqs)

        # 3. Generator impedance (analytical)
        Z_gen = self.compute_generator_impedance(bus_idx, freqs)

        return freqs, Z_total, Z_network, Z_gen

    def plot_all_impedances(self, freqs, Z_total, Z_network, Z_gen, bus_name, output_file):
        """Plot all three impedances in subplots."""
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(3, 2, figsize=(14, 12))
        fig.suptitle(f'Impedance Analysis at {bus_name}', fontsize=14)

        impedances = [
            (Z_total, 'Full Thevenin Impedance (Measured)', 'b'),
            (Z_network, 'Network Impedance (Analytical)', 'g'),
            (Z_gen, 'Generator Impedance (Analytical)', 'r')
        ]

        for row, (Z, title, color) in enumerate(impedances):
            # Magnitude
            axes[row, 0].loglog(freqs, np.abs(Z), color + '-', linewidth=2)
            axes[row, 0].set_ylabel('|Z| (pu)')
            axes[row, 0].set_title(f'{title} - Magnitude')
            axes[row, 0].grid(True, which='both', alpha=0.4)
            axes[row, 0].set_xlim(0.1, self.f_end)

            # Phase
            axes[row, 1].semilogx(freqs, np.degrees(np.angle(Z)), color + '-', linewidth=2)
            axes[row, 1].set_ylabel('Phase (°)')
            axes[row, 1].set_title(f'{title} - Phase')
            axes[row, 1].grid(True, which='both', alpha=0.4)
            axes[row, 1].set_xlim(0.1, self.f_end)
            axes[row, 1].set_ylim(-180, 180)

        axes[2, 0].set_xlabel('Frequency (Hz)')
        axes[2, 1].set_xlabel('Frequency (Hz)')

        plt.tight_layout()
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"\n  Combined impedance plot saved to: {output_file}")

        return fig