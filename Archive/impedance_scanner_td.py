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
        Uses the same approach as system_coordinator.solve_network() with full network.

        Args:
            gen_states: Generator states (n_gen x 7)
            target_bus: Bus index for injection (-1 for no injection)
            I_inj_dq: Complex injection current in DQ FRAME (Id + j*Iq)
                      This will be transformed to network (RI) frame using delta.

        Returns:
            Vd, Vq, Id, Iq: DQ frame voltages and currents
            V_term: Network frame (RI) terminal voltages
        """
        Y_full = self.coordinator.Ybus_base.copy()
        deltas = gen_states[:, 0]

        # Get generator internal admittances from metadata (already on system base)
        y_gen_internal = np.zeros(self.n_gen, dtype=complex)
        for i in range(self.n_gen):
            meta = self.builder.gen_metadata[i]
            Xd2_sys = meta.get('xd2', 0.0278)  # Already on system base
            y_gen_internal[i] = 1.0 / (1j * Xd2_sys)

        # Build internal voltages from flux linkages (E'' in machine frame)
        E_internal = np.zeros(self.n_gen, dtype=complex)
        for i in range(self.n_gen):
            meta = self.builder.gen_metadata[i]
            psi_f = gen_states[i, 4] if gen_states.shape[1] > 4 else 1.0
            psi_kq = gen_states[i, 6] if gen_states.shape[1] > 6 else 0

            xd2 = meta.get('xd2', 0.0278)
            xd1 = meta.get('xd1', 0.033)
            xl = meta.get('xl', 0.0067)
            xq2 = meta.get('xq2', 0.0278)
            xq1 = meta.get('xq1', 0.061)

            # E''_q = (xd'' - xl)/(xd' - xl) * psi_f
            if (xd1 - xl) > 1e-6:
                Eq2 = (xd2 - xl) / (xd1 - xl) * psi_f
            else:
                Eq2 = psi_f

            # E''_d from q-axis damper
            if (xq1 - xl) > 1e-6:
                Ed2 = -(xq2 - xl) / (xq1 - xl) * psi_kq
            else:
                Ed2 = 0.0

            E_internal[i] = Ed2 + 1j * Eq2

        # Transform internal voltages to system frame (dq -> RI)
        E_sys = np.zeros(self.n_gen, dtype=complex)
        for i in range(self.n_gen):
            e_d = np.real(E_internal[i])
            e_q = np.imag(E_internal[i])
            delta = deltas[i]
            E_sys[i] = (e_d * np.sin(delta) + e_q * np.cos(delta)) + \
                      1j * (-e_d * np.cos(delta) + e_q * np.sin(delta))

        # Build augmented Y matrix: Y_full + generator admittances on diagonal
        Y_aug = Y_full.copy()
        for i in range(self.n_gen):
            gen_bus = self.coordinator.gen_bus_internal[i]
            Y_aug[gen_bus, gen_bus] += y_gen_internal[i]

        # Current injection from generators: I_inj = Y_gen * E
        I_inj = np.zeros(self.coordinator.n_bus, dtype=complex)
        for i in range(self.n_gen):
            gen_bus = self.coordinator.gen_bus_internal[i]
            I_inj[gen_bus] += y_gen_internal[i] * E_sys[i]

        # Add external current injection at target bus
        if target_bus >= 0 and target_bus < self.n_gen and I_inj_dq != 0j:
            delta_target = deltas[target_bus]
            gen_bus_target = self.coordinator.gen_bus_internal[target_bus]
            Id_inj = np.real(I_inj_dq)
            Iq_inj = np.imag(I_inj_dq)
            # Transform dq injection to network RI frame
            I_inj_R = Id_inj * np.sin(delta_target) + Iq_inj * np.cos(delta_target)
            I_inj_I = -Id_inj * np.cos(delta_target) + Iq_inj * np.sin(delta_target)
            I_inj[gen_bus_target] += I_inj_R + 1j * I_inj_I

        # Iterative solution with constant power loads
        V_bus = np.ones(self.coordinator.n_bus, dtype=complex)

        for iteration in range(10):
            # Calculate load current injection (constant power model)
            I_load = np.zeros(self.coordinator.n_bus, dtype=complex)
            for lb in range(self.coordinator.n_bus):
                P = self.coordinator.load_P[lb]
                Q = self.coordinator.load_Q[lb]
                if abs(P) > 1e-6 or abs(Q) > 1e-6:
                    S_load = P + 1j * Q
                    if abs(V_bus[lb]) > 0.1:
                        I_load[lb] = np.conj(S_load / V_bus[lb])

            # Total current: generator injection minus load
            I_total = I_inj - I_load

            # Solve for bus voltages
            try:
                V_bus_new = np.linalg.solve(Y_aug, I_total)
            except np.linalg.LinAlgError:
                break

            # Check convergence
            if np.max(np.abs(V_bus_new - V_bus)) < 1e-6:
                V_bus = V_bus_new
                break
            V_bus = V_bus_new

        # Get terminal voltage for each generator
        V_term = np.zeros(self.n_gen, dtype=complex)
        for i in range(self.n_gen):
            gen_bus = self.coordinator.gen_bus_internal[i]
            V_term[i] = V_bus[gen_bus]

        # Calculate generator output current: I = Y_gen * (E - V)
        I_out = y_gen_internal * (E_sys - V_term)

        # Inverse Park transformation (system to machine frame)
        Vd = np.zeros(self.n_gen)
        Vq = np.zeros(self.n_gen)
        Id = np.zeros(self.n_gen)
        Iq = np.zeros(self.n_gen)

        for i in range(self.n_gen):
            delta = deltas[i]
            V_R = np.real(V_term[i])
            V_I = np.imag(V_term[i])
            I_R = np.real(I_out[i])
            I_I = np.imag(I_out[i])

            # Inverse Park: X_d = X_R*sin(delta) - X_I*cos(delta)
            #               X_q = X_R*cos(delta) + X_I*sin(delta)
            Vd[i] = V_R * np.sin(delta) - V_I * np.cos(delta)
            Vq[i] = V_R * np.cos(delta) + V_I * np.sin(delta)
            Id[i] = I_R * np.sin(delta) - I_I * np.cos(delta)
            Iq[i] = I_R * np.cos(delta) + I_I * np.sin(delta)

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
        """Initialize equilibrium from power flow results in JSON.

        Uses direct calculation from power flow:
        - V, theta from bus data
        - P, Q from generator data
        - Calculate E'' and delta from: E'' = V + jXd'' * I
        """
        print("  Finding equilibrium from power flow...")

        system_data = self.builder.system_data
        gen_to_bus = self.coordinator.gen_to_bus

        # Get bus voltage data
        bus_data = {b['idx']: b for b in system_data.get('Bus', [])}
        slack_data = {s['bus']: s for s in system_data.get('Slack', [])}
        pv_data = {p['bus']: p for p in system_data.get('PV', [])}

        # Arrays for results
        delta_vals = np.zeros(self.n_gen)
        psi_f_vals = np.zeros(self.n_gen)
        V_mag = np.zeros(self.n_gen)
        V_ang = np.zeros(self.n_gen)
        P_gen = np.zeros(self.n_gen)
        Q_gen = np.zeros(self.n_gen)
        Id = np.zeros(self.n_gen)
        Iq = np.zeros(self.n_gen)
        Vd = np.zeros(self.n_gen)
        Vq = np.zeros(self.n_gen)

        for i in range(self.n_gen):
            meta = self.builder.gen_metadata[i]
            bus_idx = gen_to_bus[i]

            # Get terminal voltage from power flow
            if bus_idx in bus_data:
                V_mag[i] = bus_data[bus_idx].get('v0', 1.0)
                V_ang[i] = bus_data[bus_idx].get('a0', 0.0)
            else:
                V_mag[i] = 1.0
                V_ang[i] = 0.0

            # Get power from power flow
            if bus_idx in slack_data:
                P_gen[i] = slack_data[bus_idx].get('p0', 0.0)
                Q_gen[i] = slack_data[bus_idx].get('q0', 0.0)
            elif bus_idx in pv_data:
                P_gen[i] = pv_data[bus_idx].get('p0', 0.0)
                Q_gen[i] = pv_data[bus_idx].get('q0', 0.0)
            else:
                P_gen[i] = 7.0  # Default fallback
                Q_gen[i] = 1.0

            # Generator parameters (already on system base in metadata)
            Xd2 = meta.get('xd2', 0.0278)
            ra = meta.get('ra', 0.0)

            # Terminal voltage phasor
            V_phasor = V_mag[i] * np.exp(1j * V_ang[i])

            # Current from power: I = (P - jQ)* / V* = (P + jQ) / V
            S = P_gen[i] + 1j * Q_gen[i]
            I_phasor = np.conj(S / V_phasor)

            # Internal voltage: E'' = V + (ra + jXd'') * I
            Z_gen = ra + 1j * Xd2
            E_phasor = V_phasor + Z_gen * I_phasor

            # Rotor angle delta is the angle of E''
            delta_vals[i] = np.angle(E_phasor)

            # E'' magnitude relates to psi_f through gd1 coefficient
            E_mag = np.abs(E_phasor)
            gd1 = meta.get('gd1', 0.79)
            psi_f_vals[i] = E_mag / gd1

            # Transform V and I to machine (dq) frame
            delta = delta_vals[i]
            V_R = np.real(V_phasor)
            V_I = np.imag(V_phasor)
            I_R = np.real(I_phasor)
            I_I = np.imag(I_phasor)

            Vd[i] = V_R * np.sin(delta) - V_I * np.cos(delta)
            Vq[i] = V_R * np.cos(delta) + V_I * np.sin(delta)
            Id[i] = I_R * np.sin(delta) - I_I * np.cos(delta)
            Iq[i] = I_R * np.cos(delta) + I_I * np.sin(delta)

        # Print equilibrium diagnostics
        print(f"    Equilibrium: V={V_mag}, P={P_gen}")
        print(f"    Angles (deg): {np.degrees(delta_vals)}")

        # Build full state vector
        x0 = np.zeros((self.n_gen, self.states_per_machine))

        for i in range(self.n_gen):
            meta = self.builder.gen_metadata[i]
            omega_b = meta['omega_b']
            M = meta['M']
            ra = meta['ra']

            # Calculate flux linkages from terminal conditions
            psi_d = (Vq[i] + ra * Iq[i]) / omega_b
            psi_q = -(Vd[i] + ra * Id[i]) / omega_b

            # Generator states
            x0[i, 0] = delta_vals[i]
            x0[i, 1] = M * 1.0  # p = M * omega (omega=1.0 at equilibrium)
            x0[i, 2] = psi_d
            x0[i, 3] = psi_q
            x0[i, 4] = psi_f_vals[i]
            x0[i, 5] = 0.0  # psi_kd
            x0[i, 6] = 0.0  # psi_kq

            # Exciter states
            x0[i, 7] = V_mag[i]  # vm
            x0[i, 8] = psi_f_vals[i]  # vr1 (Efd at equilibrium)

            # Clip Efd to saturation limits
            exc_meta = self.builder.exc_metadata[i]
            VRMAX = exc_meta.get('VRMAX', 5.2)
            VRMIN = exc_meta.get('VRMIN', -4.16)
            x0[i, 8] = np.clip(x0[i, 8], VRMIN, VRMAX)

            x0[i, 9] = 0.0  # vr2
            x0[i, 10] = 0.0  # vf

            # Governor states
            x0[i, 11] = P_gen[i]  # x1
            x0[i, 12] = P_gen[i]  # x2
            self.builder.gov_metadata[i]['Pref'] = P_gen[i]

            # Update Vref to maintain equilibrium Efd
            # At equilibrium: d(Efd)/dt = 0 => KA*(Vref - Vt) = Efd
            # So: Vref = Vt + Efd/KA
            exc_core = self.builder.exciters[i]
            KA = self._extract_param(exc_core, 'KA', 200.0)
            Efd_eq = x0[i, 8]  # Equilibrium field voltage
            Vref_correct = V_mag[i] + Efd_eq / KA

            for key in exc_core.subs.keys():
                if 'Vref' in str(key):
                    exc_core.subs[key] = Vref_correct
                    break

        return x0.flatten()

    def run_scan(self, bus_idx, f_max=20.0, amplitude=0.01, duration=60.0):
        """Run Scan with White Noise Injection

        KEY FIX: We run TWO simulations:
        1. BASELINE: No injection (to capture natural system response)
        2. INJECTION: With white noise injection

        The difference captures the DYNAMIC response to injection, including
        how generator flux states (psi_f, psi_kd, psi_kq) evolve differently.
        """
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

        # ========================================================
        # STEP 1: BASELINE SIMULATION (no injection)
        # ========================================================
        print(f"\n  [1/2] Running BASELINE simulation (no injection)...")
        print(f"    Total points: {len(self._inj_time):,}")

        last_print = [0]
        start_time = [__import__('time').time()]

        def progress_callback_baseline(t, x):
            current_time = __import__('time').time()
            if current_time - last_print[0] > 1.0:
                progress_pct = 100 * t / duration
                elapsed = current_time - start_time[0]
                eta = (elapsed / t * duration - elapsed) if t > 0 else 0
                bar_length = 40
                filled = int(bar_length * t / duration)
                bar = '#' * filled + '-' * (bar_length - filled)
                print(f'\r    [{bar}] {progress_pct:5.1f}% | {t:6.1f}/{duration:.0f}s | ETA: {eta/60:4.1f} min', end='', flush=True)
                last_print[0] = current_time
            return 1
        progress_callback_baseline.terminal = False

        # Run baseline (target_bus=-1 means no injection)
        sol_baseline = solve_ivp(
            lambda t, x: self._dynamics_with_injection(t, x, -1),  # -1 = no injection
            (0, duration), x_start, t_eval=self._inj_time, method='RK45',
            max_step=1.0/self.fs, events=progress_callback_baseline
        )

        bar = '#' * 40
        elapsed_baseline = __import__('time').time() - start_time[0]
        print(f'\r    [{bar}] 100.0% | {duration:6.1f}/{duration:.0f}s | Done: {elapsed_baseline/60:4.1f} min')

        # ========================================================
        # STEP 2: INJECTION SIMULATION
        # ========================================================
        print(f"\n  [2/2] Running INJECTION simulation (target bus: {bus_idx})...")

        last_print[0] = 0
        start_time[0] = __import__('time').time()

        def progress_callback_injection(t, x):
            current_time = __import__('time').time()
            if current_time - last_print[0] > 1.0:
                progress_pct = 100 * t / duration
                elapsed = current_time - start_time[0]
                eta = (elapsed / t * duration - elapsed) if t > 0 else 0
                bar_length = 40
                filled = int(bar_length * t / duration)
                bar = '#' * filled + '-' * (bar_length - filled)
                print(f'\r    [{bar}] {progress_pct:5.1f}% | {t:6.1f}/{duration:.0f}s | ETA: {eta/60:4.1f} min', end='', flush=True)
                last_print[0] = current_time
            return 1
        progress_callback_injection.terminal = False

        sol = solve_ivp(
            lambda t, x: self._dynamics_with_injection(t, x, bus_idx),
            (0, duration), x_start, t_eval=self._inj_time, method='RK45',
            max_step=1.0/self.fs, events=progress_callback_injection
        )

        bar = '#' * 40
        elapsed_injection = __import__('time').time() - start_time[0]
        print(f'\r    [{bar}] 100.0% | {duration:6.1f}/{duration:.0f}s | Done: {elapsed_injection/60:4.1f} min')
        print('    [OK] Both simulations complete!')

        # Store baseline for TFE processing
        self._sol_baseline = sol_baseline

        # Extract comprehensive signals (from injection simulation)
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

        We measure Z_dq = deltaV_dq / I_dq where both are in the machine's dq frame.

        KEY FIX: We compute voltage PERTURBATION by comparing TWO TRAJECTORIES:
        - sol: Full simulation WITH injection (states evolved with injection)
        - sol_baseline: Full simulation WITHOUT injection (states evolved naturally)
        - deltaV = V(sol, t) - V(sol_baseline, t)

        This captures the DYNAMIC frequency-dependent response, including how
        generator flux linkages (psi_f, psi_kd, psi_kq) evolved differently
        due to the injection current.
        """
        print("  Processing Transfer Function Estimate (DQ Frame)...")
        print("    Using TRAJECTORY DIFFERENCE method (captures dynamic response)")
        t = sol.t

        # Check if baseline simulation exists
        if not hasattr(self, '_sol_baseline') or self._sol_baseline is None:
            print("  ! WARNING: No baseline simulation available!")
            print("    Falling back to algebraic difference (will only capture Xd'')")
            return self._post_process_tfe_algebraic(sol, bus_idx)

        sol_base = self._sol_baseline

        deltaV_dq = np.zeros(len(t), dtype=complex)  # Voltage PERTURBATION
        I_inj_dq = np.zeros(len(t), dtype=complex)

        for k in range(len(t)):
            I_inj = self._get_injection_current_interp(t[k])
            I_inj_dq[k] = I_inj  # Injection in DQ frame

            # --- INJECTION SIMULATION: Voltage with evolved states ---
            x_inj = sol.y[:, k].reshape(self.n_gen, self.states_per_machine)
            gen_states_inj = x_inj[:, :7]
            Vd_inj, Vq_inj, _, _, _ = self._solve_network_with_injection(gen_states_inj, bus_idx, I_inj)

            # --- BASELINE SIMULATION: Voltage with baseline states ---
            x_base = sol_base.y[:, k].reshape(self.n_gen, self.states_per_machine)
            gen_states_base = x_base[:, :7]
            Vd_base, Vq_base, _, _, _ = self._solve_network_with_injection(gen_states_base, -1, 0j)

            # Voltage PERTURBATION = difference between trajectories
            deltaVd = Vd_inj[bus_idx] - Vd_base[bus_idx]
            deltaVq = Vq_inj[bus_idx] - Vq_base[bus_idx]
            deltaV_dq[k] = deltaVd + 1j * deltaVq

        # Diagnostics
        dv_rms = np.std(deltaV_dq)
        i_inj_rms = np.std(I_inj_dq)
        print(f"  > Signal Statistics (DQ Frame - Trajectory Difference):")
        print(f"      deltaV_dq_rms={dv_rms:.6f} pu (dynamic voltage perturbation)")
        print(f"      I_inj_rms={i_inj_rms:.6f} pu (injection current)")
        if i_inj_rms > 1e-9:
            print(f"      Quick Z estimate: deltaV/I = {dv_rms/i_inj_rms:.4f} pu")

        if i_inj_rms < 1e-9:
            print("  ! WARNING: Injection current is effectively zero.")

        if dv_rms < 1e-6:
            print(f"  ! WARNING: Voltage perturbation is very small ({dv_rms:.6f} pu)!")
            print(f"    Consider increasing injection amplitude.")

        # Detrend signals (remove any residual DC)
        V_ac = signal.detrend(deltaV_dq, type='constant')
        I_ac = signal.detrend(I_inj_dq, type='constant')

        # Calculate Spectra using Welch method
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

        # Impedance Z = V/I (cross-spectral density method)
        P_ii[np.abs(P_ii) < 1e-15] = 1e-15
        Z_est = P_iv / P_ii

        # Coherence (quality metric)
        P_vv[np.abs(P_vv) < 1e-15] = 1e-15
        Coherence = (np.abs(P_iv)**2) / (P_ii * P_vv)

        avg_coh = np.mean(Coherence)
        print(f"  > Average Coherence: {avg_coh:.4f}")

        if avg_coh < 0.5:
            print("    ! Low coherence indicates noisy measurement or nonlinear effects")

        # Filter by frequency range
        valid_mask = (freqs < self.f_end)

        return freqs[valid_mask], Z_est[valid_mask]

    def _post_process_tfe_algebraic(self, sol, bus_idx):
        """Fallback method: algebraic impedance (only captures Xd'')."""
        t = sol.t

        deltaV_dq = np.zeros(len(t), dtype=complex)
        I_inj_dq = np.zeros(len(t), dtype=complex)

        for k in range(len(t)):
            I_inj = self._get_injection_current_interp(t[k])
            I_inj_dq[k] = I_inj

            x_k = sol.y[:, k].reshape(self.n_gen, self.states_per_machine)
            gen_states = x_k[:, :7]

            Vd_inj, Vq_inj, _, _, _ = self._solve_network_with_injection(gen_states, bus_idx, I_inj)
            Vd_base, Vq_base, _, _, _ = self._solve_network_with_injection(gen_states, -1, 0j)

            deltaVd = Vd_inj[bus_idx] - Vd_base[bus_idx]
            deltaVq = Vq_inj[bus_idx] - Vq_base[bus_idx]
            deltaV_dq[k] = deltaVd + 1j * deltaVq

        V_ac = signal.detrend(deltaV_dq, type='constant')
        I_ac = signal.detrend(I_inj_dq, type='constant')

        n_seg = min(4096, len(t))
        f, P_iv = signal.csd(I_ac, V_ac, fs=self.fs, nperseg=n_seg, return_onesided=False)
        f, P_ii = signal.welch(I_ac, fs=self.fs, nperseg=n_seg, return_onesided=False)

        f_shift = np.fft.fftshift(f)
        P_iv_shift = np.fft.fftshift(P_iv)
        P_ii_shift = np.fft.fftshift(P_ii)

        pos_mask = f_shift > 0
        freqs = f_shift[pos_mask]
        P_iv = P_iv_shift[pos_mask]
        P_ii = P_ii_shift[pos_mask]

        P_ii[np.abs(P_ii) < 1e-15] = 1e-15
        Z_est = P_iv / P_ii

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
        Y_network = self.coordinator.Ybus_base.copy()

        # Add all generators EXCEPT the target bus (using metadata values)
        for i in range(self.n_gen):
            if i != bus_idx:
                meta = self.builder.gen_metadata[i]
                Xd2_sys = meta.get('xd2', 0.0278)  # Already on system base
                y_gen = 1.0 / (1j * Xd2_sys)
                gen_bus = self.coordinator.gen_bus_internal[i]
                Y_network[gen_bus, gen_bus] += y_gen

        # Get internal bus index for target
        gen_bus_target = self.coordinator.gen_bus_internal[bus_idx]

        # Network impedance at target bus (self-impedance from Z = inv(Y))
        Z_network_matrix = np.linalg.inv(Y_network)
        Z_network_self = Z_network_matrix[gen_bus_target, gen_bus_target]

        # Network impedance is frequency-independent (purely algebraic)
        # Return same value for all frequencies
        Z_network = np.full(len(freqs), Z_network_self, dtype=complex)

        print(f"      Z_network = {np.abs(Z_network_self):.4f} pu @ {np.degrees(np.angle(Z_network_self)):.1f} deg")

        return Z_network

    def compute_generator_impedance(self, bus_idx, freqs):
        """Compute synchronous generator impedance vs frequency.

        The generator impedance is the operational impedance of the synchronous
        machine. For GENROU model, the d-axis operational impedance is:

        Zd(s) = Ra + Xd''*s * (1 + s*Td'') * (1 + s*Td') / ((1 + s*Td0') * (1 + s*Td0''))

        At different frequencies:
        - f -> 0: Z -> Ra (stator resistance)
        - f = 60 Hz: Z ~ j*Xd'' (subtransient reactance)
        - f -> inf: Z -> Ra + j*w*Ll (leakage)

        Note: Values in metadata are ALREADY on system base (100 MVA).
        """
        print(f"  Computing Generator Impedance for Gen {bus_idx+1}...")

        meta = self.builder.gen_metadata[bus_idx]
        gen_core = self.builder.generators[bus_idx]

        # Machine parameters (ALREADY on system base in metadata)
        Ra = meta['ra']
        Xd = meta.get('xd', 0.2)
        Xd1 = meta['xd1']
        Xd2 = meta.get('xd2', 0.0278)
        Xl = meta['xl']

        # Time constants from component
        Td10 = self._extract_param(gen_core, 'Td10', 8.0)
        Td20 = self._extract_param(gen_core, 'Td20', 0.03)

        # Derived time constants (short-circuit)
        if Xd > 1e-6:
            Td1 = Td10 * Xd1 / Xd
        else:
            Td1 = Td10
        if Xd1 > 1e-6:
            Td2 = Td20 * Xd2 / Xd1
        else:
            Td2 = Td20

        print(f"      Ra={Ra:.4f}, Xd={Xd:.4f}, Xd'={Xd1:.4f}, Xd''={Xd2:.4f} pu (system base)")
        print(f"      Td0'={Td10:.2f}s, Td0''={Td20:.3f}s, Td'={Td1:.4f}s, Td''={Td2:.6f}s")

        # Compute frequency-dependent impedance
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