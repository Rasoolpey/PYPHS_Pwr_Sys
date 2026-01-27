"""
Impedance Scanner - Frequency Domain Analysis for PHS
"""
import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import solve
from utils.system_builder import PowerSystemBuilder
from utils.system_coordinator import PowerSystemCoordinator

class ImpedanceScanner:
    """
    Scans the impedance spectrum of the power system at a specific bus
    by linearizing the Port-Hamiltonian dynamics.
    """
    
    def __init__(self, json_file):
        self.builder = PowerSystemBuilder(json_file)
        self.builder.build_all_components()
        self.coordinator = PowerSystemCoordinator(self.builder)
        
        # System dimensions
        self.n_gen = len(self.builder.generators)
        self.states_per_machine = 13
        self.total_states = self.n_gen * self.states_per_machine
        
        # Store equilibrium
        self.x0 = None
        self.y_bus_solver = None

    def list_buses(self):
        """Return list of available buses for scanning"""
        # In this reduced model, we can only scan generator buses
        buses = []
        for i, meta in enumerate(self.builder.gen_metadata):
            buses.append({
                'idx': i,
                'name': f"Gen {i+1} (Bus {meta['bus']})",
                'bus_id': meta['bus']
            })
        return buses

    def find_equilibrium(self):
        """
        Find equilibrium using power flow results from JSON.

        Uses direct calculation from power flow:
        - V, theta from bus data
        - P, Q from generator data
        - Calculate E'' and delta from: E'' = V + jXd'' * I
        """
        print("Computing equilibrium from power flow...")

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

        # Verify power calculation
        P_check = Vd * Id + Vq * Iq
        print("  Power verification:")
        for i in range(self.n_gen):
            print(f"    Gen {i}: P_target={P_gen[i]:.3f}, P_calc={P_check[i]:.3f}, diff={abs(P_gen[i]-P_check[i]):.5f}")

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
            exc_m = self.builder.exc_metadata[i]
            VRMAX = exc_m.get('VRMAX', 5.2)
            VRMIN = exc_m.get('VRMIN', -4.16)
            x0[i, 8] = np.clip(x0[i, 8], VRMIN, VRMAX)

            x0[i, 9] = 0.0  # vr2
            x0[i, 10] = 0.0  # vf

            # Governor states
            x0[i, 11] = P_gen[i]  # x1
            x0[i, 12] = P_gen[i]  # x2
            self.builder.gov_metadata[i]['Pref'] = P_gen[i]

        print("\n  Equilibrium:")
        for i in range(self.n_gen):
            print(f"    Gen {i}: delta={np.degrees(delta_vals[i]):.2f} deg, P={P_gen[i]:.3f} pu, V={V_mag[i]:.4f} pu")

        # Verify solve_network consistency
        print("\n  Verifying solve_network consistency...")
        gen_states = x0[:, :7]
        Id_net, Iq_net, Vd_net, Vq_net = self.coordinator.solve_network(gen_states)
        P_net = Vd_net * Id_net + Vq_net * Iq_net

        for i in range(self.n_gen):
            P_diff = abs(P_gen[i] - P_net[i])
            print(f"    Gen {i}: P_init={P_gen[i]:.3f}, P_net={P_net[i]:.3f}, diff={P_diff:.4f}")

        # Verify stability with short simulation
        from scipy.integrate import solve_ivp
        sol = solve_ivp(lambda t, x: self._dynamics_wrapper(x, perturbation=None),
                       (0, 5.0), x0.flatten(), method='RK45', max_step=0.1)

        x_final = sol.y[:, -1].reshape(self.n_gen, self.states_per_machine)
        delta_drift = np.rad2deg(np.max(np.abs(x_final[:, 0] - x0[:, 0])))
        print(f"\n  Equilibrium stability check: delta_drift = {delta_drift:.2f} deg over 5s")

        if delta_drift > 3.0:
            print("  WARNING: Equilibrium may be unstable!")

        self.x0 = x0.flatten()
        print("Equilibrium found.")
        return self.x0

    def _solve_network_with_injection_v2(self, gen_states, target_bus, Id_inj, Iq_inj):
        """
        Network solver with current injection support for D-matrix computation.
        Uses the same approach as system_coordinator.solve_network() but with injection.

        IMPORTANT: xd2 in metadata is ALREADY on system base (100 MVA).
        """
        Y_full = self.coordinator.Ybus_base.copy()

        # Extract rotor angles
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
        if target_bus >= 0 and target_bus < self.n_gen:
            delta_target = deltas[target_bus]
            gen_bus_target = self.coordinator.gen_bus_internal[target_bus]
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

        return Vd, Vq, Id, Iq
    
    def _extract_param(self, component_core, param_name, default_value):
        """Extract parameter from component's substitution dictionary"""
        for key, value in component_core.subs.items():
            if param_name in str(key):
                return float(value)
        return default_value

    def _dynamics_wrapper(self, x_flat, perturbation=None):
        """
        Internal dynamics function wrapped to accept current injections (perturbations).
        
        Args:
            x_flat: Flattened state vector
            perturbation: dict {'bus_idx': int, 'Id_inj': float, 'Iq_inj': float}
        """
        x = x_flat.reshape(self.n_gen, self.states_per_machine)
        gen_states = x[:, :7]
        exc_states = x[:, 7:11]
        gov_states = x[:, 11:13]
        
        # Network Solve with optional current injection
        if perturbation:
            bus_idx = perturbation['bus_idx']
            Id_inj = perturbation.get('Id_inj', 0.0)
            Iq_inj = perturbation.get('Iq_inj', 0.0)
            Vd, Vq, Id, Iq = self._solve_network_with_injection_v2(gen_states, bus_idx, Id_inj, Iq_inj)
        else:
            Id, Iq, Vd, Vq = self.coordinator.solve_network(gen_states)

        # Dynamics calculation
        dxdt = np.zeros((self.n_gen, self.states_per_machine))
        
        for i in range(self.n_gen):
            meta = self.builder.gen_metadata[i]
            gen_core = self.builder.generators[i]
            
            # Get voltages and currents for this generator
            vd_val = Vd[i]
            vq_val = Vq[i]
            id_val = Id[i]
            iq_val = Iq[i]
            
            # --- Generator Physics ---
            M = meta['M']
            D = meta['D']
            ra = meta['ra']
            omega_b = meta['omega_b']
            
            delta, p, psi_d, psi_q, psi_f, psi_kd, psi_kq = gen_states[i]
            omega = p / M
            
            # Governor Physics
            x1, x2 = gov_states[i]
            gov_meta = self.builder.gov_metadata[i]
            Tm = (gov_meta['T2']/gov_meta['T3']) * (x1 - x2) + x2 - gov_meta['Dt'] * (omega - 1.0)
            
            # Generator Dynamics
            Te = vd_val * id_val + vq_val * iq_val
            dxdt[i, 0] = omega_b * (omega - 1.0)
            dxdt[i, 1] = Tm - Te - D * (omega - 1.0)
            dxdt[i, 2] = vd_val - ra * id_val + omega_b * omega * psi_q
            dxdt[i, 3] = vq_val - ra * iq_val - omega_b * omega * psi_d
            
            # Flux Dynamics
            Td10 = [v for k, v in gen_core.subs.items() if 'Td10' in str(k)][0]
            Td20 = [v for k, v in gen_core.subs.items() if 'Td20' in str(k)][0]
            Tq20 = [v for k, v in gen_core.subs.items() if 'Tq20' in str(k)][0]
            Efd = exc_states[i, 1]
            
            dxdt[i, 4] = (Efd - psi_f) / Td10
            dxdt[i, 5] = -psi_kd / Td20
            dxdt[i, 6] = -psi_kq / Tq20
            
            # --- Exciter Physics ---
            Vt = np.sqrt(vd_val**2 + vq_val**2)
            vm, vr1, vr2, vf = exc_states[i]
            exc_core = self.builder.exciters[i]
            TR = [v for k, v in exc_core.subs.items() if 'TR' in str(k)][0]
            TA = [v for k, v in exc_core.subs.items() if 'TA' in str(k)][0]
            KA = [v for k, v in exc_core.subs.items() if 'KA' in str(k)][0]
            Vref = [v for k, v in exc_core.subs.items() if 'Vref' in str(k)][0]
            
            dxdt[i, 7] = (Vt - vm) / TR
            Verr = Vref - vm
            
            # CRITICAL FIX: Add exciter saturation (linearize around saturated state)
            exc_meta = self.builder.exc_metadata[i]
            VRMAX = exc_meta.get('VRMAX', 5.2)
            VRMIN = exc_meta.get('VRMIN', -4.16)
            
            vr1_unlimited = (KA * Verr - vr1) / TA
            
            # Anti-windup saturation logic
            if vr1 >= VRMAX and vr1_unlimited > 0:
                dxdt[i, 8] = 0.0  # Saturated high, no further increase
            elif vr1 <= VRMIN and vr1_unlimited < 0:
                dxdt[i, 8] = 0.0  # Saturated low, no further decrease
            else:
                dxdt[i, 8] = vr1_unlimited
            
            dxdt[i, 9] = 0.0
            dxdt[i, 10] = 0.0
            
            # --- Governor Dynamics ---
            gate_cmd = gov_meta['Pref'] + (gov_meta['wref'] - omega) / gov_meta['R']
            
            # Add governor saturation limits
            gate_limited = np.clip(gate_cmd, gov_meta.get('VMIN', 0.0), gov_meta.get('VMAX', 10.0))
            dxdt[i, 11] = 10.0 * (gate_limited - x1) / gov_meta['T1']
            dxdt[i, 12] = 10.0 * (x1 - x2) / gov_meta['T3']
            
        return dxdt.flatten()

    def get_linear_model(self, bus_idx):
        """
        Compute A, B, C, D matrices via numerical perturbation.
        State vector x: System states
        Input vector u: [Id_inj, Iq_inj] at bus_idx
        Output vector y: [Vd, Vq] at bus_idx
        """
        print(f"Linearizing system at Bus {bus_idx}...")
        
        if self.x0 is None:
            self.find_equilibrium()
            
        epsilon = 1e-5
        x0_flat = self.x0
        f0 = self._dynamics_wrapper(x0_flat)
        n = len(x0_flat)
        
        # 1. Compute A Matrix (Jacobian df/dx)
        A = np.zeros((n, n))
        for i in range(n):
            x_pert = x0_flat.copy()
            x_pert[i] += epsilon
            f_pert = self._dynamics_wrapper(x_pert)
            A[:, i] = (f_pert - f0) / epsilon
            
        # 2. Compute B Matrix (Input matrix df/du)
        # Inputs: u[0]=Id_injection, u[1]=Iq_injection
        B = np.zeros((n, 2))
        
        # Perturb Id injection
        f_pert_id = self._dynamics_wrapper(x0_flat, {'bus_idx': bus_idx, 'Id_inj': epsilon})
        B[:, 0] = (f_pert_id - f0) / epsilon
        
        # Perturb Iq injection
        f_pert_iq = self._dynamics_wrapper(x0_flat, {'bus_idx': bus_idx, 'Iq_inj': epsilon})
        B[:, 1] = (f_pert_iq - f0) / epsilon
        
        # 3. Compute C, D Matrices (Output Equation y = Cx + Du)
        # Outputs: y[0]=Vd, y[1]=Vq at bus_idx
        
        # Get baseline voltages
        x0_reshaped = x0_flat.reshape(self.n_gen, self.states_per_machine)
        gen_states = x0_reshaped[:, :7]
        _, _, Vd0, Vq0 = self.coordinator.solve_network(gen_states)
        y0 = np.array([Vd0[bus_idx], Vq0[bus_idx]])
        
        C = np.zeros((2, n))
        D = np.zeros((2, 2))
        
        # Perturb states for C
        for i in range(n):
            x_pert = x0_flat.copy()
            x_pert[i] += epsilon
            x_r = x_pert.reshape(self.n_gen, self.states_per_machine)
            _, _, Vd_p, Vq_p = self.coordinator.solve_network(x_r[:, :7])
            y_p = np.array([Vd_p[bus_idx], Vq_p[bus_idx]])
            C[:, i] = (y_p - y0) / epsilon
            
        # Perturb inputs for D - FIX: Properly compute D-matrix
        # D represents immediate voltage change due to current injection (algebraic path)
        # D = dV/dI at constant states (network impedance)
        
        # Method: Numerically perturb current injection and measure voltage change
        # while keeping all states constant (algebraic network response only)
        
        # We need a modified network solver that accepts current injection
        # Use the same approach as IMTB scanner
        
        # Perturb Id injection (keep states fixed at x0)
        Vd_baseline, Vq_baseline, _, _ = self._solve_network_with_injection_v2(
            gen_states, bus_idx, 0.0, 0.0)
        Vd_pert_id, Vq_pert_id, _, _ = self._solve_network_with_injection_v2(
            gen_states, bus_idx, epsilon, 0.0)
        
        D[0, 0] = (Vd_pert_id[bus_idx] - Vd_baseline[bus_idx]) / epsilon  # dVd/dId
        D[1, 0] = (Vq_pert_id[bus_idx] - Vq_baseline[bus_idx]) / epsilon  # dVq/dId
        
        # Perturb Iq injection (keep states fixed at x0)
        Vd_pert_iq, Vq_pert_iq, _, _ = self._solve_network_with_injection_v2(
            gen_states, bus_idx, 0.0, epsilon)
        
        D[0, 1] = (Vd_pert_iq[bus_idx] - Vd_baseline[bus_idx]) / epsilon  # dVd/dIq
        D[1, 1] = (Vq_pert_iq[bus_idx] - Vq_baseline[bus_idx]) / epsilon  # dVq/dIq
        
        print(f"  D-matrix computed (network impedance at infinite frequency):")
        print(f"    D = {D}")
        
        return A, B, C, D

    def scan_impedance(self, bus_idx, freq_range_hz):
        """
        Perform frequency sweep
        
        Args:
            bus_idx: Index of bus to scan
            freq_range_hz: Array of frequencies in Hz
            
        Returns:
            freqs: Frequency array
            Z_mag: Magnitude of impedance
            Z_phase: Phase of impedance
            Z_dq: Full DQ impedance matrix (N_freq x 2 x 2)
        """
        A, B, C, D = self.get_linear_model(bus_idx)
        
        Z_dq = np.zeros((len(freq_range_hz), 2, 2), dtype=complex)
        
        print(f"Scanning {len(freq_range_hz)} frequency points...")
        I = np.eye(len(A))
        
        for i, f in enumerate(freq_range_hz):
            omega = 2 * np.pi * f
            s = 1j * omega
            
            # Resolvent: (sI - A)^-1
            # Z(s) = C * (sI - A)^-1 * B + D
            try:
                Resolvent = solve(s*I - A, B) # More numerically stable than inv
                Z_mat = C @ Resolvent + D
                Z_dq[i, :, :] = Z_mat
            except np.linalg.LinAlgError:
                Z_dq[i, :, :] = np.nan
                
        # Extract dq impedance components
        Z_dd = Z_dq[:, 0, 0]  # d-axis self-impedance (mostly resistive)
        Z_qq = Z_dq[:, 1, 1]  # q-axis self-impedance (mostly resistive)
        Z_dq_cross = Z_dq[:, 0, 1]  # d to q coupling (contains -Xq)
        Z_qd_cross = Z_dq[:, 1, 0]  # q to d coupling (contains +Xd)

        # For synchronous machines in dq frame:
        # V_d = ra*I_d - Xq*I_q + E_d  =>  Z_dd ~ ra, Z_dq ~ -Xq
        # V_q = ra*I_q + Xd*I_d + E_q  =>  Z_qq ~ ra, Z_qd ~ +Xd
        # The dominant impedance is in the cross-coupling terms

        # Compute positive-sequence impedance (more useful for stability analysis)
        # Z_pos = (Z_dd + Z_qq)/2 + j*(Z_qd - Z_dq)/2
        Z_pos = (Z_dd + Z_qq)/2 + 1j*(Z_qd_cross - Z_dq_cross)/2

        # Print some diagnostic values
        print(f"\n  Impedance Diagnostics:")
        print(f"    DC (0.1 Hz):  |Z_qd| = {np.abs(Z_qd_cross[0]):.3f} pu, |Z_pos| = {np.abs(Z_pos[0]):.3f} pu")
        mid_idx = len(freq_range_hz) // 2
        print(f"    Mid ({freq_range_hz[mid_idx]:.1f} Hz): |Z_qd| = {np.abs(Z_qd_cross[mid_idx]):.3f} pu, |Z_pos| = {np.abs(Z_pos[mid_idx]):.3f} pu")
        print(f"    High ({freq_range_hz[-1]:.1f} Hz): |Z_qd| = {np.abs(Z_qd_cross[-1]):.3f} pu, |Z_pos| = {np.abs(Z_pos[-1]):.3f} pu")
        print(f"    D-matrix: D[1,0]={D[1,0]:.4f} (should be ~Xd''={self.builder.gen_metadata[bus_idx].get('xd2', 0.028):.4f})\n")

        # Use positive-sequence impedance magnitude for main plot
        Z_mag = np.abs(Z_pos)
        Z_phase = np.degrees(np.angle(Z_pos))
        
        return freq_range_hz, Z_mag, Z_phase, Z_dq