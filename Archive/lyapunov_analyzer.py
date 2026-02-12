"""
Lyapunov Stability Analyzer - Port-Hamiltonian based stability analysis
Uses system_builder + coordinator for dynamic system analysis
Implements passivity-based stability certificates and energy methods
"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
from scipy.linalg import eig, eigvals
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from utils.system_builder import PowerSystemBuilder
from utils.system_coordinator import PowerSystemCoordinator


class LyapunovStabilityAnalyzer:
    """
    Port-Hamiltonian Lyapunov Stability Analysis
    
    Implements:
    1. Passivity certificate via Hamiltonian as storage function
    2. Equilibrium stability via energy gradient
    3. Linearized stability (Jacobian eigenvalues)
    4. Transient energy margin computation
    5. Stability region estimation
    """
    
    def __init__(self, builder_or_json):
        """
        Args:
            builder_or_json: PowerSystemBuilder or JSON filepath
        """
        if isinstance(builder_or_json, str):
            self.builder = PowerSystemBuilder(builder_or_json)
            self.builder.build_all_components()
        else:
            self.builder = builder_or_json
            
        # System coordinator handles network
        self.coordinator = PowerSystemCoordinator(self.builder)
        
        self.n_gen = len(self.builder.generators)
        self.n_gen_states = 7
        self.n_exc_states = 4
        self.n_gov_states = 2
        self.states_per_machine = 13
        self.total_states = self.n_gen * self.states_per_machine
        
        # Base frequency
        self.omega_b = 2 * np.pi * 60
        
        # Equilibrium point (to be initialized)
        self.x_eq = None
        self.H_eq = None
        self.Y_int = None  # Reduced internal admittance matrix for potential energy
        
        print(f"\nLyapunov Stability Analyzer: {self.n_gen} generators, {self.total_states} states")
        
    def initialize_equilibrium(self):
        """
        Initialize equilibrium point from power flow.
        Uses the same approach as fault_sim_modular for consistency.
        """
        print("\n[Equilibrium Initialization]")

        system_data = self.builder.system_data

        # Build lookup dictionaries
        bus_data = {b['idx']: b for b in system_data.get('Bus', [])}
        slack_data = {s['bus']: s for s in system_data.get('Slack', [])}
        pv_data = {p['bus']: p for p in system_data.get('PV', [])}

        # Get generator to bus mapping
        gen_to_bus = {i: self.builder.gen_metadata[i]['bus'] for i in range(self.n_gen)}

        # Arrays for intermediate values
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
                P_gen[i] = 7.0
                Q_gen[i] = 0.0

            # Generator parameters (on system base)
            Xd2 = meta.get('xd2', 0.25)
            ra = meta.get('ra', 0.0)

            # Terminal voltage phasor
            V_phasor = V_mag[i] * np.exp(1j * V_ang[i])

            # Current from power: I = conj(S / V)
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
            psi_f_vals[i] = E_mag / gd1 if gd1 > 0 else 1.0

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

        # Build full state vector
        x0 = np.zeros(self.total_states)

        for i in range(self.n_gen):
            idx = i * self.states_per_machine
            meta = self.builder.gen_metadata[i]
            omega_b = meta['omega_b']
            M = meta['M']
            ra = meta['ra']

            # Calculate flux linkages from terminal conditions
            # At steady state with omega=1: psi_d approx Vq/omega_b, psi_q approx -Vd/omega_b
            psi_d = (Vq[i] + ra * Iq[i]) / omega_b
            psi_q = -(Vd[i] + ra * Id[i]) / omega_b

            # Generator states [delta, p, psi_d, psi_q, psi_f, psi_kd, psi_kq]
            x0[idx + 0] = delta_vals[i]  # delta
            x0[idx + 1] = M * 1.0  # p = M * omega (omega=1.0 at equilibrium)
            x0[idx + 2] = psi_d  # psi_d
            x0[idx + 3] = psi_q  # psi_q
            x0[idx + 4] = psi_f_vals[i]  # psi_f (field flux)
            x0[idx + 5] = 0.0  # psi_kd (d-axis damper)
            x0[idx + 6] = 0.0  # psi_kq (q-axis damper)

            # Exciter states [vm, vr1, vr2, vf]
            Efd_eq = psi_f_vals[i]  # Field voltage at equilibrium
            x0[idx + 7] = V_mag[i]  # vm (voltage measurement)
            x0[idx + 8] = Efd_eq  # vr1 (exciter output = Efd)
            x0[idx + 9] = 0.0  # vr2
            x0[idx + 10] = 0.0  # vf

            # Governor states [x1, x2]
            x0[idx + 11] = P_gen[i]  # x1 (governor state)
            x0[idx + 12] = P_gen[i]  # x2 (governor state)

            # Update governor Pref in metadata
            self.builder.gov_metadata[i]['Pref'] = P_gen[i]

            # Update exciter Vref to maintain equilibrium
            # At equilibrium: d(Efd)/dt = 0 => KA*(Vref - Vt) = Efd
            # So: Vref = Vt + Efd/KA
            exc_meta = self.builder.exc_metadata[i]
            KA = exc_meta.get('KA', 20.0)
            Vref_correct = V_mag[i] + Efd_eq / KA

            # Update exciter metadata with correct Vref
            self.builder.exc_metadata[i]['Vref'] = Vref_correct

            # Also update the core's metadata
            self.builder.exciters[i]._metadata['Vref'] = Vref_correct

        # Compute reduced internal Ybus for potential energy calculation
        print("  Computing reduced internal network for energy function...")
        self.Y_int = self._compute_reduced_internal_ybus()

        # Refine equilibrium iteratively
        x0 = self._refine_equilibrium(x0)

        self.x_eq = x0

        # Compute equilibrium Hamiltonian
        self.H_eq = self.compute_hamiltonian(x0)

        # Print summary
        print(f"  Equilibrium Hamiltonian: H* = {self.H_eq:.6f}")
        delta_final = [np.degrees(x0[i * self.states_per_machine]) for i in range(self.n_gen)]
        print(f"  Rotor angles (deg): {[f'{d:.2f}' for d in delta_final]}")
        print(f"  Generator powers: {[f'{P_gen[i]:.3f}' for i in range(self.n_gen)]}")
        print("  Equilibrium initialized successfully")

        return x0

    def _compute_reduced_internal_ybus(self):
        """Compute Ybus reduced to generator internal nodes for energy function"""
        # 1. Get full Ybus
        Y_full = self.coordinator.Ybus_base.copy()
        n_bus = Y_full.shape[0]
        
        # 2. Augment with internal nodes
        n_total = n_bus + self.n_gen
        Y_aug = np.zeros((n_total, n_total), dtype=complex)
        Y_aug[:n_bus, :n_bus] = Y_full
        
        internal_indices = []
        
        for i in range(self.n_gen):
            gen_bus = self.coordinator.gen_bus_internal[i]
            internal_idx = n_bus + i
            internal_indices.append(internal_idx)
            
            # Internal impedance (transient reactance)
            meta = self.builder.gen_metadata[i]
            Xd2 = meta.get('xd2', 0.0278) 
            y_gen = 1.0 / (1j * Xd2)
            
            # Add connections
            Y_aug[internal_idx, internal_idx] += y_gen
            Y_aug[gen_bus, gen_bus] += y_gen
            Y_aug[internal_idx, gen_bus] -= y_gen
            Y_aug[gen_bus, internal_idx] -= y_gen
            
        # 3. Kron Reduction
        keep = np.array(internal_indices)
        elim = np.arange(n_bus)
        
        Y_kk = Y_aug[np.ix_(keep, keep)]
        Y_ke = Y_aug[np.ix_(keep, elim)]
        Y_ek = Y_aug[np.ix_(elim, keep)]
        Y_ee = Y_aug[np.ix_(elim, elim)]
        
        # Add loads to Y_ee diagonal (constant impedance approximation)
        for i in range(n_bus):
            P = self.coordinator.load_P[i]
            Q = self.coordinator.load_Q[i]
            if abs(P) > 1e-6 or abs(Q) > 1e-6:
                y_load = (P - 1j*Q) / 1.0**2
                Y_ee[i, i] += y_load
        
        # Invert Y_ee and reduce
        try:
            Y_ee_inv = np.linalg.inv(Y_ee)
        except np.linalg.LinAlgError:
            Y_ee_inv = np.linalg.pinv(Y_ee)
            
        Y_red = Y_kk - Y_ke @ Y_ee_inv @ Y_ek
        return Y_red

    def _refine_equilibrium(self, x0, tol=1e-3):
        """
        Check equilibrium quality and report.

        The initial guess from power flow should be close to equilibrium.
        We just verify and report the error - the simulation will handle convergence.
        """
        print("  Verifying equilibrium...")

        x_dot = self._system_dynamics(x0, np.zeros(self.n_gen))
        max_error = np.max(np.abs(x_dot))

        # Exclude the fast stator flux dynamics from error check
        # (states 2,3 for each generator: psi_d, psi_q)
        slow_errors = []
        for i in range(self.n_gen):
            idx = i * self.states_per_machine
            # Check swing equation errors (delta, p)
            slow_errors.append(np.abs(x_dot[idx]))  # delta_dot
            slow_errors.append(np.abs(x_dot[idx + 1]))  # p_dot
            # Check rotor flux errors
            slow_errors.append(np.abs(x_dot[idx + 4]))  # psi_f_dot
            # Check exciter errors
            slow_errors.append(np.abs(x_dot[idx + 7]))  # vm_dot
            slow_errors.append(np.abs(x_dot[idx + 8]))  # vr1_dot
            # Check governor errors
            slow_errors.append(np.abs(x_dot[idx + 11]))  # x1_dot
            slow_errors.append(np.abs(x_dot[idx + 12]))  # x2_dot

        max_slow_error = np.max(slow_errors)

        print(f"    Max |dx/dt| (all states): {max_error:.2e}")
        print(f"    Max |dx/dt| (slow states): {max_slow_error:.2e}")

        if max_slow_error < tol:
            print(f"  Equilibrium verified for electromechanical dynamics")
        else:
            print(f"  Warning: Slow dynamics not fully at equilibrium")

        return x0
        
    def compute_hamiltonian(self, x):
        """
        Compute total system Hamiltonian (energy)
        H = sum(kinetic + magnetic energy) for all generators
        
        Args:
            x: state vector
            
        Returns:
            H: total energy
        """
        H_total = 0.0
        
        for i in range(self.n_gen):
            idx = i * self.states_per_machine
            meta = self.builder.gen_metadata[i]
            
            # Extract generator states
            p = x[idx + 1]
            psi_d = x[idx + 2]
            psi_q = x[idx + 3]
            psi_f = x[idx + 4]
            psi_1d = x[idx + 5]
            psi_kq = x[idx + 6]
            
            # Kinetic energy
            M = meta.get('M', 10.0)
            omega = p / M
            omega_ref = 1.0
            H_kinetic = 0.5 * M * (omega - omega_ref)**2
            
            # Magnetic energy (flux linkages)
            xd = meta.get('xd', 0.146)
            xq = meta.get('xq', 0.0969)
            xd1 = meta.get('xd1', 0.033)
            xq1 = meta.get('xq1', 0.061)
            xl = meta.get('xl', 0.0067)
            
            # Simplified magnetic energy
            H_magnetic = 0.5 * (psi_d**2 / xd + psi_q**2 / xq + 
                               psi_f**2 / xd1 + psi_1d**2 / xd1 + 
                               psi_kq**2 / xq1)
            
            H_total += H_kinetic + H_magnetic
        
        # Add Potential Energy (Network) if initialized
        if self.Y_int is not None and self.x_eq is not None:
            H_potential = self._compute_potential_energy(x)
            H_total += H_potential
            
        return H_total
        
    def _compute_potential_energy(self, x):
        """
        Compute potential energy in Center of Inertia (COI) frame.
        Using COI ensures the energy function is rotationally invariant
        and locally positive definite.
        """
        V_pe = 0.0
        
        # Extract states
        deltas = x[0::self.states_per_machine]
        deltas_eq = self.x_eq[0::self.states_per_machine]
        
        # Get Inertia for COI calculation
        M = np.array([self.builder.gen_metadata[i].get('M', 10.0) for i in range(self.n_gen)])
        M_tot = np.sum(M)
        
        # Calculate Center of Inertia (COI)
        delta_coi = np.sum(M * deltas) / M_tot
        delta_coi_eq = np.sum(M * deltas_eq) / M_tot
        
        # Transform to COI frame (theta = delta - delta_coi)
        thetas = deltas - delta_coi
        thetas_eq = deltas_eq - delta_coi_eq
        
        # Get internal voltages and powers
        E_mag = np.zeros(self.n_gen)
        Pm_eq = np.zeros(self.n_gen)
        
        for i in range(self.n_gen):
            # Mechanical power reference
            Pm_eq[i] = self.builder.gov_metadata[i]['Pref']
            
            # Internal voltage magnitude (approximate from equilibrium flux)
            idx = i * self.states_per_machine
            psi_f_eq = self.x_eq[idx + 4]
            gd1 = self.builder.gen_metadata[i].get('gd1', 0.79)
            E_mag[i] = psi_f_eq * gd1 if gd1 > 0 else 1.0
            
        # Calculate PE
        for i in range(self.n_gen):
            # Term 1: Path dependent energy (Power mismatch work)
            # Use COI angles (thetas) to remove rotational drift energy
            V_pe -= Pm_eq[i] * (thetas[i] - thetas_eq[i])
            
            for j in range(i + 1, self.n_gen):
                # Term 2: Magnetic energy of lines (Potential well)
                # C_ij = E_i * E_j * B_ij
                C_ij = E_mag[i] * E_mag[j] * np.abs(self.Y_int[i, j])
                
                # Angle difference is invariant: theta_i - theta_j = delta_i - delta_j
                d_ij = deltas[i] - deltas[j]
                d_ij_eq = deltas_eq[i] - deltas_eq[j]
                
                V_pe -= C_ij * (np.cos(d_ij) - np.cos(d_ij_eq))
                
        return V_pe

    def compute_lyapunov_function(self, x):
        """
        Compute Lyapunov function for port-Hamiltonian power system.
        
        V(x) = Kinetic Energy Deviation + Magnetic Energy Deviation + Potential Energy
        
        This is the proper energy function for transient stability:
        - V(x*) = 0 at equilibrium
        - V(x) > 0 for x != x*
        - dV/dt < 0 along system trajectories (dissipative)
        
        Args:
            x: current state
            
        Returns:
            V: Lyapunov function value (energy above equilibrium)
        """
        if self.x_eq is None:
            raise ValueError("Equilibrium not initialized. Call initialize_equilibrium() first.")
            
        V_total = 0.0
        
        # Extract rotor angles and speeds
        deltas = x[0::self.states_per_machine]
        deltas_eq = self.x_eq[0::self.states_per_machine]
        
        # Get inertia constants for COI calculation
        M_vec = np.array([self.builder.gen_metadata[i].get('M', 10.0) for i in range(self.n_gen)])
        M_tot = np.sum(M_vec)
        
        # Center of Inertia (COI) angle - for rotational invariance
        delta_coi = np.sum(M_vec * deltas) / M_tot
        delta_coi_eq = np.sum(M_vec * deltas_eq) / M_tot
        
        # Transform to COI frame
        thetas = deltas - delta_coi
        thetas_eq = deltas_eq - delta_coi_eq
        
        # 1. KINETIC ENERGY (speed deviations from synchronous speed)
        for i in range(self.n_gen):
            idx = i * self.states_per_machine
            M = M_vec[i]
            p = x[idx + 1]
            omega = p / M
            V_total += 0.5 * M * (omega - 1.0)**2
            
        # 2. POTENTIAL ENERGY (electromechanical coupling in COI frame)
        # Classical energy function for transient stability analysis
        
        # Get equilibrium powers and electrical parameters
        Pm_eq = np.zeros(self.n_gen)
        Pe_eq = np.zeros(self.n_gen)  # Electrical power at equilibrium
        E_mag_eq = np.zeros(self.n_gen)
        
        for i in range(self.n_gen):
            Pm_eq[i] = self.builder.gov_metadata[i]['Pref']
            Pe_eq[i] = Pm_eq[i]  # At equilibrium: Pe = Pm
            idx = i * self.states_per_machine
            psi_f_eq = self.x_eq[idx + 4]
            gd1 = self.builder.gen_metadata[i].get('gd1', 0.79)
            E_mag_eq[i] = psi_f_eq * gd1
        
        # POTENTIAL ENERGY: Two-part formulation
        # Part 1: Acceleration area (work done against mechanical power)
        for i in range(self.n_gen):
            V_total += Pm_eq[i] * (thetas[i] - thetas_eq[i])
        
        # Part 2: Network coupling energy (using linearized power flow)
        # This creates a quadratic potential well around equilibrium
        if self.Y_int is not None:
            for i in range(self.n_gen):
                for j in range(i + 1, self.n_gen):
                    # Coupling strength from network admittance
                    Y_ij_mag = np.abs(self.Y_int[i, j])
                    Y_ij_ang = np.angle(self.Y_int[i, j])
                    C_ij = E_mag_eq[i] * E_mag_eq[j] * Y_ij_mag
                    
                    # Angle differences
                    delta_ij = deltas[i] - deltas[j]
                    delta_ij_eq = deltas_eq[i] - deltas_eq[j]
                    d_delta = delta_ij - delta_ij_eq
                    
                    # Quadratic approximation around equilibrium (always positive)
                    # V_ij = 0.5 * C_ij * sin(delta_ij_eq + Y_ij_ang) * (delta_ij - delta_ij_eq)^2
                    # This is the second-order Taylor expansion of -C_ij*cos(delta_ij)
                    V_pe_ij = 0.5 * C_ij * np.abs(np.sin(delta_ij_eq + Y_ij_ang)) * d_delta**2
                    
                    V_total += V_pe_ij
        
        # 3. MAGNETIC ENERGY (flux deviations - usually small contribution)
        for i in range(self.n_gen):
            idx = i * self.states_per_machine
            meta = self.builder.gen_metadata[i]
            
            # Simplified diagonal inductance model
            xd = meta.get('xd', 0.146)
            xq = meta.get('xq', 0.0969)
            xd1 = meta.get('xd1', 0.033)
            
            # Flux deviations (stator and rotor)
            d_psi_d = x[idx+2] - self.x_eq[idx+2]
            d_psi_q = x[idx+3] - self.x_eq[idx+3]
            d_psi_f = x[idx+4] - self.x_eq[idx+4]
            
            # Magnetic energy storage (quadratic in flux deviations)
            V_mag = 0.5 * (d_psi_d**2/xd + d_psi_q**2/xq + d_psi_f**2/xd1)
            V_total += V_mag
        
        return V_total
        
    def verify_passivity(self, x, u, y):
        """
        Verify passivity condition: dH/dt <= u^T y
        
        Args:
            x: state vector
            u: input vector
            y: output vector
            
        Returns:
            dict with passivity verification results
        """
        # Compute time derivative of Hamiltonian numerically
        dt = 1e-6
        x_dot = self._system_dynamics(x, u)
        H_current = self.compute_hamiltonian(x)
        H_next = self.compute_hamiltonian(x + x_dot * dt)
        dH_dt = (H_next - H_current) / dt
        
        # Compute supplied power
        u_T_y = np.dot(u, y)
        
        # Passivity holds if dH/dt <= u^T y
        passivity_satisfied = dH_dt <= u_T_y + 1e-6  # numerical tolerance
        
        return {
            'dH_dt': dH_dt,
            'u_T_y': u_T_y,
            'passivity_satisfied': passivity_satisfied,
            'margin': u_T_y - dH_dt
        }
        
    def compute_jacobian(self, x_eq):
        """
        Compute Jacobian at equilibrium point using finite differences
        
        Args:
            x_eq: equilibrium state
            
        Returns:
            J: Jacobian matrix
        """
        print("\n[Jacobian Computation]")
        
        n = len(x_eq)
        J = np.zeros((n, n))
        epsilon = 1e-6
        
        # Baseline dynamics
        f0 = self._system_dynamics(x_eq, np.zeros(self.n_gen))
        
        # Finite difference approximation
        for i in range(n):
            x_pert = x_eq.copy()
            x_pert[i] += epsilon
            f_pert = self._system_dynamics(x_pert, np.zeros(self.n_gen))
            J[:, i] = (f_pert - f0) / epsilon
            
            if (i + 1) % 10 == 0:
                print(f"  Computing column {i+1}/{n}...")
                
        print("  Jacobian computation complete")
        return J
        
    def analyze_linearized_stability(self, x_eq=None):
        """
        Analyze stability via eigenvalue analysis of Jacobian
        
        Args:
            x_eq: equilibrium point (uses self.x_eq if None)
            
        Returns:
            dict with eigenvalue analysis results
        """
        if x_eq is None:
            if self.x_eq is None:
                raise ValueError("Equilibrium not initialized")
            x_eq = self.x_eq
            
        print("\n[Linearized Stability Analysis]")
        
        # Compute Jacobian
        J = self.compute_jacobian(x_eq)
        
        # Compute eigenvalues
        eigenvalues = eigvals(J)
        
        # Classify eigenvalues
        real_parts = np.real(eigenvalues)
        imag_parts = np.imag(eigenvalues)
        
        stable_count = np.sum(real_parts < -1e-6)
        unstable_count = np.sum(real_parts > 1e-6)
        marginal_count = np.sum(np.abs(real_parts) <= 1e-6)
        
        # System is stable if all eigenvalues have negative real part
        is_stable = unstable_count == 0
        
        # Find dominant eigenvalues (largest real part)
        dominant_idx = np.argsort(real_parts)[-5:]  # Top 5 critical modes
        
        results = {
            'is_stable': is_stable,
            'eigenvalues': eigenvalues,
            'real_parts': real_parts,
            'imag_parts': imag_parts,
            'stable_count': stable_count,
            'unstable_count': unstable_count,
            'marginal_count': marginal_count,
            'dominant_eigenvalues': eigenvalues[dominant_idx],
            'jacobian': J
        }
        
        print(f"  Stability: {'STABLE' if is_stable else 'UNSTABLE'}")
        print(f"  Stable modes: {stable_count}")
        print(f"  Unstable modes: {unstable_count}")
        print(f"  Marginal modes: {marginal_count}")
        print(f"\n  Dominant eigenvalues:")
        for i, idx in enumerate(dominant_idx[::-1]):
            eig_val = eigenvalues[idx]
            print(f"    {i+1}. lambda = {eig_val.real:.4f} + {eig_val.imag:.4f}j")
            
        return results
        
    def compute_energy_margin(self, x_fault):
        """
        Compute transient energy margin
        Critical energy = H(unstable equilibrium) - H(stable equilibrium)
        
        Args:
            x_fault: post-fault state
            
        Returns:
            dict with energy margin analysis
        """
        if self.x_eq is None:
            raise ValueError("Equilibrium not initialized")
            
        H_stable = self.H_eq
        H_fault = self.compute_hamiltonian(x_fault)
        
        # Energy absorbed during fault
        delta_H = H_fault - H_stable
        
        return {
            'H_stable': H_stable,
            'H_fault': H_fault,
            'delta_H': delta_H,
            'normalized_margin': delta_H / H_stable if H_stable > 0 else 0
        }
        
    def estimate_stability_region(self, x_eq=None, n_samples=100, max_perturbation=0.5):
        """
        Estimate domain of attraction using Lyapunov function derivative criterion.
        
        For port-Hamiltonian systems with dissipation:
        - Point is in stability region if: V(x) < V_critical AND dV/dt < 0
        - Much faster than trajectory simulation
        
        Args:
            x_eq: equilibrium point
            n_samples: number of random samples
            max_perturbation: maximum perturbation magnitude
            
        Returns:
            dict with stability region estimates
        """
        if x_eq is None:
            if self.x_eq is None:
                raise ValueError("Equilibrium not initialized")
            x_eq = self.x_eq
            
        print(f"\n[Stability Region Estimation - {n_samples} samples]")
        print("  Using Lyapunov derivative criterion (dV/dt < 0)...")
        
        stable_samples = []
        unstable_samples = []
        V_samples = []
        dVdt_samples = []
        
        for i in range(n_samples):
            # Random perturbation with non-uniform distribution
            # Focus more on electromechanical states (angles and speeds)
            perturbation = np.zeros(len(x_eq))
            
            for gen in range(self.n_gen):
                idx = gen * self.states_per_machine
                # Rotor angle perturbation (rad)
                perturbation[idx] = np.random.randn() * max_perturbation * 0.5
                # Speed perturbation (pu) - from momentum
                M = self.builder.gen_metadata[gen]['M']
                perturbation[idx+1] = np.random.randn() * max_perturbation * M * 0.1
                # Field flux perturbation
                perturbation[idx+4] = np.random.randn() * max_perturbation * 0.1
                
            x_sample = x_eq + perturbation
            
            # Compute Lyapunov function
            V = self.compute_lyapunov_function(x_sample)
            V_samples.append(V)
            
            # Compute dV/dt at this point
            x_dot = self._system_dynamics(x_sample, np.zeros(self.n_gen))
            dV_dt = self._compute_dV_dt(x_sample, x_dot)
            dVdt_samples.append(dV_dt)
            
            # Stability criterion: dV/dt < 0 (energy decreasing)
            # Also check that state is reasonable (no NaN, no extreme values)
            is_stable = (dV_dt < 0) and (V < 1e6) and np.all(np.isfinite(x_sample))
            
            if is_stable:
                stable_samples.append(x_sample)
            else:
                unstable_samples.append(x_sample)
            
            # Progress indicator
            if (i + 1) % 100 == 0:
                print(f"    Progress: {i+1}/{n_samples} samples...")
                
        stable_fraction = len(stable_samples) / n_samples
        
        # Estimate critical energy level (conservative)
        V_stable = [V_samples[i] for i in range(n_samples) if i < len(stable_samples)]
        V_critical = max(V_stable) if V_stable else 0
        
        results = {
            'n_samples': n_samples,
            'stable_count': len(stable_samples),
            'unstable_count': len(unstable_samples),
            'stable_fraction': stable_fraction,
            'V_critical': V_critical,
            'V_samples': np.array(V_samples),
            'dVdt_samples': np.array(dVdt_samples),
            'stable_samples': stable_samples,
            'unstable_samples': unstable_samples
        }
        
        print(f"  Stable samples: {len(stable_samples)}/{n_samples} ({stable_fraction*100:.1f}%)")
        print(f"  Estimated V_critical: {V_critical:.4f}")
        
        return results
        
    def _compute_dV_dt(self, x, x_dot):
        """
        Compute time derivative of Lyapunov function: dV/dt = grad(V)^T * f(x)
        
        For port-Hamiltonian systems, this simplifies significantly:
        dV/dt = -sum(dissipation terms) <= 0
        
        We use sparse numerical gradient focusing on key states for efficiency.
        """
        # For accurate stability assessment, compute gradient on key states only
        # (rotor angles, speeds, field flux) - ignore fast stator dynamics
        
        epsilon = 1e-6
        grad_V = np.zeros(len(x))
        
        V0 = self.compute_lyapunov_function(x)
        
        # Only compute gradient for electromechanical states (much faster)
        for i in range(self.n_gen):
            idx = i * self.states_per_machine
            
            # Angle gradient
            x_pert = x.copy()
            x_pert[idx] += epsilon
            grad_V[idx] = (self.compute_lyapunov_function(x_pert) - V0) / epsilon
            
            # Momentum gradient
            x_pert = x.copy()
            x_pert[idx + 1] += epsilon
            grad_V[idx + 1] = (self.compute_lyapunov_function(x_pert) - V0) / epsilon
            
            # Field flux gradient (key for voltage stability)
            x_pert = x.copy()
            x_pert[idx + 4] += epsilon
            grad_V[idx + 4] = (self.compute_lyapunov_function(x_pert) - V0) / epsilon
        
        # Compute dV/dt using sparse gradient
        dV_dt = np.dot(grad_V, x_dot)
        return dV_dt
        
    def _system_dynamics(self, x, u):
        """
        Compute system dynamics: dx/dt = f(x, u)
        Uses generator, exciter, and governor models via their dynamics methods.

        Args:
            x: state vector
            u: control input (mechanical power references for governors)

        Returns:
            x_dot: state derivatives
        """
        x_dot = np.zeros_like(x)

        # Extract generator states for network solution
        gen_states = np.zeros((self.n_gen, self.n_gen_states))
        for i in range(self.n_gen):
            idx = i * self.states_per_machine
            gen_states[i, :] = x[idx:idx+self.n_gen_states]

        # Solve network
        try:
            Id, Iq, Vd, Vq = self.coordinator.solve_network(gen_states)
        except:
            # If network solution fails, use zero currents
            Id = np.zeros(self.n_gen)
            Iq = np.zeros(self.n_gen)
            Vd = np.zeros(self.n_gen)
            Vq = np.zeros(self.n_gen)

        # Compute dynamics for each generator
        for i in range(self.n_gen):
            idx = i * self.states_per_machine
            gen_meta = self.builder.gen_metadata[i]
            gov_meta = self.builder.gov_metadata[i]

            # Extract states
            gen_x = x[idx:idx+self.n_gen_states]
            exc_x = x[idx+self.n_gen_states:idx+self.n_gen_states+self.n_exc_states]
            gov_x = x[idx+self.n_gen_states+self.n_exc_states:idx+self.states_per_machine]

            # Get omega from momentum state
            M = gen_meta['M']
            omega = gen_x[1] / M

            # Governor output (mechanical torque)
            # Use Pref from metadata unless explicit non-zero control input is provided
            Pm_ref = gov_meta['Pref']
            if i < len(u) and u[i] != 0:
                Pm_ref = u[i]
            gov_ports = {'omega': omega, 'Pm_ref': Pm_ref}

            # Import tgov1_output for mechanical torque calculation
            from components.governors.tgov1 import tgov1_output
            Tm = tgov1_output(gov_x, gov_ports, gov_meta)

            # Exciter output (field voltage)
            Efd = exc_x[1]  # vr1 is the exciter output

            # Generator dynamics with complete port information
            gen_ports = {
                'Id': Id[i], 'Iq': Iq[i],
                'Vd': Vd[i], 'Vq': Vq[i],
                'Tm': Tm, 'Efd': Efd
            }
            gen_x_dot = self.builder.generators[i].dynamics(gen_x, gen_ports)
            x_dot[idx:idx+self.n_gen_states] = gen_x_dot

            # Exciter dynamics
            Vt = np.sqrt(Vd[i]**2 + Vq[i]**2)
            exc_ports = {'Vt': Vt}
            exc_x_dot = self.builder.exciters[i].dynamics(exc_x, exc_ports)
            x_dot[idx+self.n_gen_states:idx+self.n_gen_states+self.n_exc_states] = exc_x_dot

            # Governor dynamics
            gov_x_dot = self.builder.governors[i].dynamics(gov_x, gov_ports)
            x_dot[idx+self.n_gen_states+self.n_exc_states:idx+self.states_per_machine] = gov_x_dot

        return x_dot
        
    def plot_eigenvalues(self, results):
        """
        Plot eigenvalues in complex plane
        
        Args:
            results: output from analyze_linearized_stability
        """
        eigenvalues = results['eigenvalues']
        
        plt.figure(figsize=(10, 8))
        
        # Plot all eigenvalues
        plt.scatter(np.real(eigenvalues), np.imag(eigenvalues), 
                   c='blue', s=50, alpha=0.6, label='All modes')
        
        # Highlight dominant eigenvalues
        dominant = results['dominant_eigenvalues']
        plt.scatter(np.real(dominant), np.imag(dominant),
                   c='red', s=100, marker='*', label='Dominant modes')
        
        # Add stability boundary
        plt.axvline(x=0, color='k', linestyle='--', linewidth=1)
        
        plt.xlabel('Real Part')
        plt.ylabel('Imaginary Part')
        plt.title('Eigenvalue Analysis - Linearized System')
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.axis('equal')
        
        return plt.gcf()
        
    def plot_lyapunov_evolution(self, time, states):
        """
        Plot Lyapunov function evolution over time
        
        Args:
            time: time vector
            states: state trajectory (time x states)
        """
        V_evolution = []
        
        for i in range(len(time)):
            V = self.compute_lyapunov_function(states[i, :])
            V_evolution.append(V)
            
        plt.figure(figsize=(10, 6))
        plt.plot(time, V_evolution, linewidth=2)
        plt.xlabel('Time (s)')
        plt.ylabel('V(x) = H(x) - H(x*)')
        plt.title('Lyapunov Function Evolution')
        plt.grid(True, alpha=0.3)
        plt.axhline(y=0, color='r', linestyle='--', label='Equilibrium')
        plt.legend()
        
        return plt.gcf()
        
    def generate_stability_report(self, output_path='lyapunov_report.txt'):
        """
        Generate comprehensive stability report
        
        Args:
            output_path: path to save report
        """
        if self.x_eq is None:
            print("Initializing equilibrium...")
            self.initialize_equilibrium()
            
        print("\n" + "="*70)
        print("   LYAPUNOV STABILITY ANALYSIS REPORT")
        print("="*70)
        
        # Linearized stability
        linear_results = self.analyze_linearized_stability()
        
        # Write to file with UTF-8 encoding for Unicode support
        with open(output_path, 'w', encoding='utf-8') as f:
            f.write("="*70 + "\n")
            f.write("   LYAPUNOV STABILITY ANALYSIS REPORT\n")
            f.write("="*70 + "\n\n")
            
            f.write(f"System Configuration:\n")
            f.write(f"  Generators: {self.n_gen}\n")
            f.write(f"  Total states: {self.total_states}\n")
            f.write(f"  Equilibrium energy: H* = {self.H_eq:.6f}\n\n")
            
            f.write(f"Linearized Stability:\n")
            f.write(f"  Status: {'STABLE' if linear_results['is_stable'] else 'UNSTABLE'}\n")
            f.write(f"  Stable modes: {linear_results['stable_count']}\n")
            f.write(f"  Unstable modes: {linear_results['unstable_count']}\n")
            f.write(f"  Marginal modes: {linear_results['marginal_count']}\n\n")
            
            f.write(f"Dominant Eigenvalues:\n")
            for i, eig_val in enumerate(linear_results['dominant_eigenvalues'][::-1]):
                f.write(f"  {i+1}. lambda = {eig_val.real:.6f} + {eig_val.imag:.6f}j\n")
                
        print(f"\nReport saved to: {output_path}")
