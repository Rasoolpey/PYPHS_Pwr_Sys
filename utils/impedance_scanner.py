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
        Calculates the steady-state operating point x0.
        (Simplified version of the logic in fault_sim_modular)
        """
        print("Computing equilibrium point...")
        # 1. Initialize guesses
        delta_init = np.array([0.08, 0.07, 0.06, 0.07])
        psi_f_init = np.array([1.3, 1.3, 1.3, 1.3])
        
        # 2. Iterative solve (Simplified for brevity, assuming stability)
        # In a real tool, we would copy the robust solver from fault_sim_modular
        x0 = np.zeros((self.n_gen, self.states_per_machine))
        
        # ... (Using a fast-forward approximation for the scanner demo)
        # We need the exact equilibrium to linearize correctly.
        # Let's perform a quick settling simulation to find x0
        
        def settling_dynamics(t, x):
            return self._dynamics_wrapper(x, perturbation=None)

        from scipy.integrate import solve_ivp
        
        # Initial rough guess
        for i in range(4):
            x0[i, 0] = delta_init[i]
            x0[i, 1] = self.builder.gen_metadata[i]['M'] # omega=1
            x0[i, 4] = psi_f_init[i] # psi_f
            x0[i, 7] = 1.0 # Vm
            x0[i, 8] = psi_f_init[i] # Efd approx
            x0[i, 11] = 7.0 # P_mech
            x0[i, 12] = 7.0 # P_mech

        # Simulate briefly to settle
        sol = solve_ivp(settling_dynamics, (0, 5.0), x0.flatten(), method='RK45')
        self.x0 = sol.y[:, -1]
        print("Equilibrium found.")
        return self.x0

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
        
        # 1. Network Solve (Modified for Injection)
        # We need to monkey-patch or modify the coordinator solve to accept I_inj
        # Since coordinator.solve_network is hardcoded, we calculate the injection effect manually here:
        # I_inj affects the current balance equation: Y V = I_gen + I_inj - I_load
        
        # Standard solve first
        Id, Iq, Vd, Vq = self.coordinator.solve_network(gen_states)
        
        # Apply Perturbation Superposition (Linear approximation for Jacobian)
        # If we inject current at a bus, Voltage rises by Z_thevenin * I_inj
        # Note: Ideally, we pass I_inj into solve_network. 
        # For this scanner, we assume the perturbation is small and handled via
        # the numerical differentiation of the CLOSED LOOP system.
        
        # HOWEVER, to get the 'B' matrix (Input -> State), we strictly need 
        # to inject current.
        
        if perturbation:
            # We must modify the voltages Vd, Vq based on the injection.
            # This requires the inverse Ybus (Zbus).
            # V_new = V_old + Zbus * I_inj
            
            # Reconstruct full complex arrays
            bus_idx = perturbation['bus_idx']
            I_inj_complex = np.zeros(4, dtype=complex)
            # perturbation current in machine frame needs rotation to network frame?
            # For simplicity in impedance scanning, we often scan in the synchronous frame (dq).
            # Let's assume the injection is in the Network Frame (easier) or Machine Frame.
            # Let's inject in the Machine Frame of the target bus.
            
            delta = gen_states[bus_idx, 0]
            i_d = perturbation.get('Id_inj', 0)
            i_q = perturbation.get('Iq_inj', 0)
            
            # Current injection vector
            # This is a simplification. For rigorous PHS, we should inject at the port.
            # Vd[bus_idx] += i_d * R_dummy # This is just a dummy change to detect sensitivity
            # To do this correctly without rewriting coordinator, we accept Vd, Vq as "Inputs" 
            # for the partial derivative B matrix, and then close the loop with Z_network.
            pass

        # ... (Rest of dynamics calculation identical to fault_sim_modular)
        dxdt = np.zeros((self.n_gen, self.states_per_machine))
        
        # Re-implementing core physics for derivative calculation
        # (Ideally this should be a shared method in a physics module)
        for i in range(self.n_gen):
            meta = self.builder.gen_metadata[i]
            gen_core = self.builder.generators[i]
            
            # Current injection handling for B-matrix calculation
            # If we are perturbing the CURRENT at this bus, we modify Id/Iq seen by the generator
            id_val = Id[i]
            iq_val = Iq[i]
            
            if perturbation and perturbation['bus_idx'] == i:
                 id_val += perturbation.get('Id_inj', 0)
                 iq_val += perturbation.get('Iq_inj', 0)

            vd_val = Vd[i]
            vq_val = Vq[i]
            
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
            vr1_unlimited = (KA * Verr - vr1) / TA
            dxdt[i, 8] = vr1_unlimited # Ignoring limits for linearization
            dxdt[i, 9] = 0.0
            dxdt[i, 10] = 0.0
            
            # --- Governor Dynamics ---
            gate_cmd = gov_meta['Pref'] + (gov_meta['wref'] - omega) / gov_meta['R']
            dxdt[i, 11] = 10.0 * (gate_cmd - x1) / gov_meta['T1']
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
            
        # Perturb inputs for D
        # Note: Since network is algebraic, D represents the immediate change in V due to I
        # This is essentially the Thevenin impedance of the network itself at infinite frequency
        # For this simplified model, we can approximate D by checking V changes without state changes
        # However, solve_network assumes I_gen determines V. If we inject current, V changes immediately.
        # D ~ Z_thevenin
        
        # Since solve_network doesn't natively support I_inj, D is effectively 0 in this specific code structure
        # (Voltages only change if states change). 
        # A more advanced implementation would invert Ybus to find D.
        
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
        
        # Simplified magnitude for plotting
        Z_mag = np.abs(Z_dd) 
        Z_phase = np.degrees(np.angle(Z_dd))
        
        return freq_range_hz, Z_mag, Z_phase, Z_dq