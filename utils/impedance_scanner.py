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
        Find equilibrium using iterative power flow trim (same as fault_sim_modular).
        """
        print("Computing equilibrium point (iterative trim)...")
        
        # Target active powers and initial guesses
        P_targets = np.array([7.459, 7.0, 7.0, 7.0])
        delta_init = np.array([0.08, 0.07, 0.06, 0.07])
        psi_f_init = np.array([1.3, 1.3, 1.3, 1.3])
        
        # Iterative trim loop (matching fault_sim_modular)
        for iteration in range(100):  # Increase iterations
            gen_states = np.zeros((self.n_gen, 7))
            for i in range(self.n_gen):
                M = self.builder.gen_metadata[i]['M']
                gen_states[i] = [delta_init[i], M * 1.0, 0.0, 0.0, psi_f_init[i], 0.0, 0.0]
            
            Id, Iq, Vd, Vq = self.coordinator.solve_network(gen_states)
            P_calc = Vd * Id + Vq * Iq
            V_mag = np.sqrt(Vd**2 + Vq**2)
            
            max_error = 0
            for i in range(self.n_gen):
                # Adjust delta to match power
                P_error = P_targets[i] - P_calc[i]
                delta_init[i] += 0.02 * P_error
                
                # Adjust field flux to match voltage
                V_error = 1.0 - V_mag[i]
                psi_f_init[i] += 0.1 * V_error
                psi_f_init[i] = np.clip(psi_f_init[i], 0.5, 2.0)
                
                max_error = max(max_error, abs(P_error))
            
            if iteration % 10 == 0:
                print(f"  Iter {iteration}: P_error={max_error:.5f}")
            
            converged = max_error < 1e-4
            if converged:
                print(f"  Converged in {iteration+1} iterations")
                break
        
        if not converged:
            print(f"  WARNING: Did not fully converge (error={max_error:.6f})")
        
        # Build complete state vector (matching fault_sim_modular)
        x0 = np.zeros((self.n_gen, self.states_per_machine))
        for i in range(self.n_gen):
            meta = self.builder.gen_metadata[i]
            omega_b = meta['omega_b']
            M = meta['M']
            ra = meta['ra']
            
            # Calculate psi_d and psi_q from network solution
            psi_q = (-Vd[i] - ra * Id[i]) / omega_b
            psi_d = (Vq[i] + ra * Iq[i]) / omega_b
            
            x0[i, 0] = delta_init[i]
            x0[i, 1] = M * 1.0
            x0[i, 2] = psi_d
            x0[i, 3] = psi_q
            x0[i, 4] = psi_f_init[i]
            x0[i, 5] = 0.0
            x0[i, 6] = 0.0
            x0[i, 7] = np.sqrt(Vd[i]**2 + Vq[i]**2)  # Vm
            x0[i, 8] = psi_f_init[i]  # Efd = psi_f at equilibrium
            
            # Clip Efd to saturation limits
            exc_m = self.builder.exc_metadata[i]
            VRMAX = exc_m.get('VRMAX', 5.2)
            VRMIN = exc_m.get('VRMIN', -4.16)
            x0[i, 8] = np.clip(x0[i, 8], VRMIN, VRMAX)
            
            x0[i, 9] = 0.0  # vr2
            x0[i, 10] = 0.0  # vf
            
            # Governor states - use actual calculated power at equilibrium
            P_eq = P_calc[i]
            x0[i, 11] = P_eq  # x1 (gate position)
            x0[i, 12] = P_eq  # x2
            
            # Update governor metadata with equilibrium power
            self.builder.gov_metadata[i]['Pref'] = P_eq
        
        # Verify stability with short simulation
        from scipy.integrate import solve_ivp
        sol = solve_ivp(lambda t, x: self._dynamics_wrapper(x, perturbation=None),
                       (0, 5.0), x0.flatten(), method='RK45')
        
        x_final = sol.y[:, -1].reshape(self.n_gen, 13)
        delta_drift = np.rad2deg(np.max(np.abs(x_final[:, 0] - x0[:, 0])))
        print(f"  Equilibrium stability check: Δδ = {delta_drift:.2f}° over 5s")
        
        if delta_drift > 3.0:
            print("  WARNING: Equilibrium may be unstable!")
        
        self.x0 = x0.flatten()
        print("Equilibrium found.")
        return self.x0

    def _solve_network_with_injection_v2(self, gen_states, target_bus, Id_inj, Iq_inj):
        """
        Network solver with current injection support for D-matrix computation.
        Based on IMTB scanner implementation.
        """
        Ybus = self.coordinator.Ybus_base
        
        # Generator internal voltage sources (corrected xd2 scaling)
        Y_gen_diag = np.zeros(self.n_gen, dtype=complex)
        E_sys = np.zeros(self.n_gen, dtype=complex)
        
        for i in range(self.n_gen):
            meta = self.builder.gen_metadata[i]
            gen_core = self.builder.generators[i]
            
            delta = gen_states[i, 0]
            psi_d, psi_q, psi_f, psi_kd, psi_kq = gen_states[i, 2:7]
            
            # Extract parameters
            xd2_machine = self._extract_param(gen_core, 'xd2', 0.25)
            xq2_machine = self._extract_param(gen_core, 'xq2', 0.25)
            
            # CRITICAL: Scale from machine base to system base
            Sn = meta['Sn']
            S_system = self.builder.S_system
            xd2 = xd2_machine * (Sn / S_system)
            xq2 = xq2_machine * (Sn / S_system)
            
            # Machine inductance matrix (dq frame)
            xd = self._extract_param(gen_core, 'xd', 1.8)
            xq = self._extract_param(gen_core, 'xq', 1.7)
            
            Ld_eq = (xd * psi_f + xd2 * psi_kd) / (xd + xd2)
            Lq_eq = (xq * psi_kq) / (xq + xq2) if abs(xq + xq2) > 1e-9 else 0.0
            
            psi2d = psi_d - Ld_eq
            psi2q = psi_q - Lq_eq
            
            # Equivalent generator impedance
            y_gen_dq = (1.0 / (xd2 + 1j * xq2))
            Y_gen_diag[i] = y_gen_dq
            
            # Internal voltage in network frame
            E_int = psi2d + 1j * psi2q
            E_sys[i] = (np.real(E_int) * np.cos(delta) - np.imag(E_int) * np.sin(delta)) + \
                       1j * (np.real(E_int) * np.sin(delta) + np.imag(E_int) * np.cos(delta))
        
        # Current injection in network frame
        I_gen_source = E_sys * Y_gen_diag
        S_load = self.coordinator.load_P + 1j * self.coordinator.load_Q
        V_term = np.ones(self.n_gen, dtype=complex)
        Y_total = Ybus + np.diag(Y_gen_diag)
        
        I_inj_net_complex = 0j
        if target_bus >= 0:
            delta_target = gen_states[target_bus, 0]
            # Transform dq injection to network frame
            I_inj_net_complex = (Id_inj * np.cos(delta_target) - Iq_inj * np.sin(delta_target)) + \
                                1j * (Id_inj * np.sin(delta_target) + Iq_inj * np.cos(delta_target))
        
        # Two-iteration network solve (matching SystemCoordinator)
        I_load = np.conj(S_load / V_term)
        I_rhs = I_gen_source - I_load
        if target_bus >= 0:
            I_rhs[target_bus] += I_inj_net_complex
        V_term = np.linalg.solve(Y_total, I_rhs)
        
        # Refine with voltage safety clamping
        V_safe = np.where(np.abs(V_term) > 0.5, V_term, np.ones(self.n_gen, dtype=complex))
        I_load = np.conj(S_load / V_safe)
        I_rhs = I_gen_source - I_load
        if target_bus >= 0:
            I_rhs[target_bus] += I_inj_net_complex
        V_term = np.linalg.solve(Y_total, I_rhs)
        
        # Generator output current
        I_out = (E_sys - V_term) * Y_gen_diag
        
        # Transform to dq frame
        Vd = np.zeros(self.n_gen)
        Vq = np.zeros(self.n_gen)
        Id = np.zeros(self.n_gen)
        Iq = np.zeros(self.n_gen)
        
        for i in range(self.n_gen):
            delta = gen_states[i, 0]
            v_r, v_i = np.real(V_term[i]), np.imag(V_term[i])
            i_r, i_i = np.real(I_out[i]), np.imag(I_out[i])
            
            Vd[i] = v_r * np.cos(delta) + v_i * np.sin(delta)
            Vq[i] = -v_r * np.sin(delta) + v_i * np.cos(delta)
            Id[i] = i_r * np.cos(delta) + i_i * np.sin(delta)
            Iq[i] = -i_r * np.sin(delta) + i_i * np.cos(delta)
        
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
                
        # Calculate SISO equivalent impedance (Positive Sequence approx)
        # Z_pos approx (Zdd + Zqq)/2 + j(Zdq - Zqd)/2
        Z_dd = Z_dq[:, 0, 0]
        Z_qq = Z_dq[:, 1, 1]
        
        # Print some diagnostic values
        print(f"\n  Impedance Diagnostics:")
        print(f"    DC (0.1 Hz): |Z_dd| = {np.abs(Z_dd[0]):.2f} pu")
        mid_idx = len(freq_range_hz) // 2
        print(f"    Mid ({freq_range_hz[mid_idx]:.1f} Hz): |Z_dd| = {np.abs(Z_dd[mid_idx]):.2f} pu")
        print(f"    High ({freq_range_hz[-1]:.1f} Hz): |Z_dd| = {np.abs(Z_dd[-1]):.2f} pu")
        print(f"    D-matrix contribution: |D[0,0]| = {np.abs(D[0,0]):.2f} pu\n")
        
        # Simplified magnitude for plotting
        Z_mag = np.abs(Z_dd) 
        Z_phase = np.degrees(np.angle(Z_dd))
        
        return freq_range_hz, Z_mag, Z_phase, Z_dq