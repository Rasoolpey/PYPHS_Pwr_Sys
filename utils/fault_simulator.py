"""
Fault Simulator - Uses PyPHS components we built
"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from utils.system_builder import PowerSystemBuilder


class FaultSimulator:
    """Fault simulator using modular PyPHS components"""
    
    def __init__(self, json_file):
        """Initialize from JSON"""
        self.builder = PowerSystemBuilder(json_file)
        self.builder.build_all_components()
        
        self.n_gen = len(self.builder.generators)
        
        # State dimensions per component
        self.n_gen_states = 7   # [delta, p, psi_d, psi_q, psi_f, psi_kd, psi_kq]
        self.n_exc_states = 4   # [vm, vr1, vr2, vf]
        self.n_gov_states = 2   # [x1_gov, x2_gov]
        self.states_per_machine = self.n_gen_states + self.n_exc_states + self.n_gov_states
        self.total_states = self.n_gen * self.states_per_machine
        
        # Fault config
        import json
        with open(json_file, 'r') as f:
            data = json.load(f)
        net_cfg = data.get('network_config', {})
        
        self.fault_start = 1.0
        self.fault_duration = 0.1
        self.fault_end = self.fault_start + self.fault_duration
        self.fault_bus = net_cfg.get('fault_bus_idx', 2)
        
        self._build_ybus()
        
        print(f"\nFault Simulator: {self.total_states} states, fault @ Gen {self.fault_bus}")
    
    def _build_ybus(self):
        """Build Ybus matrices"""
        net_meta = self.builder.net_metadata
        X_intra = net_meta['X_intra']
        X_tie = net_meta['X_tie']
        
        y_intra = 1.0 / (1j * X_intra)
        y_tie = 1.0 / (1j * X_tie)
        
        # Normal Ybus
        self.Ybus = np.zeros((4, 4), dtype=complex)
        self.Ybus[0, 0] = y_intra + y_tie
        self.Ybus[0, 1] = -y_intra
        self.Ybus[0, 2] = -y_tie
        self.Ybus[1, 1] = y_intra
        self.Ybus[1, 0] = -y_intra
        self.Ybus[2, 2] = y_intra + y_tie
        self.Ybus[2, 3] = -y_intra
        self.Ybus[2, 0] = -y_tie
        self.Ybus[3, 3] = y_intra
        self.Ybus[3, 2] = -y_intra
        
        # Fault Ybus
        self.Ybus_fault = self.Ybus.copy()
        fault_adm = 1.0 / 0.005j
        self.Ybus_fault[self.fault_bus, self.fault_bus] += fault_adm
    
    def _solve_network(self, gen_states, mode='normal'):
        """Network solver with Park transforms"""
        deltas = gen_states[:, 0]
        Y_net = self.Ybus_fault if mode == 'fault' else self.Ybus
        
        # Gen admittance
        Xd2 = 0.25 * (100.0 / 900.0)
        y_gen = 1.0 / (1j * Xd2)
        Y_gen = np.eye(4) * y_gen
        
        # Internal voltages
        E_internal = np.zeros(4, dtype=complex)
        for i in range(4):
            meta = self.builder.gen_metadata[i]
            psi_f = gen_states[i, 4]
            psi_kd = gen_states[i, 5]
            psi_kq = gen_states[i, 6]
            
            gd1 = meta['gd1']
            gd2 = meta['gd2']
            gq1 = meta['gq1']
            xd1 = meta['xd1']
            xl = meta['xl']
            
            psi2d = gd1 * psi_f + gd2 * (xd1 - xl) * psi_kd
            psi2q = (1 - gq1) * psi_kq
            E_internal[i] = psi2d + 1j * psi2q
        
        # Forward Park
        E_sys = np.zeros(4, dtype=complex)
        for i in range(4):
            e_d = np.real(E_internal[i])
            delta = deltas[i]
            E_sys[i] = e_d * np.cos(delta) + 1j * e_d * np.sin(delta)
        
        # Network solve
        I_gen = E_sys * y_gen
        
        # Load currents - use actual Kundur load values
        # Load at bus 7: 11.59 MW split between Gen 0-1
        # Load at bus 8: 15.75 MW split between Gen 2-3
        load_P_actual = np.array([11.59/2, 11.59/2, 15.75/2, 15.75/2])
        load_Q_actual = np.array([-0.735/2, -0.735/2, -0.899/2, -0.899/2])
        
        S_load = load_P_actual + 1j * load_Q_actual
        V_est = np.ones(4, dtype=complex)
        I_load = np.conj(S_load / V_est)
        
        I_inj = I_gen - I_load
        Y_total = Y_net + Y_gen
        V_term = np.linalg.solve(Y_total, I_inj)
        
        # Refine
        V_safe = np.where(np.abs(V_term) > 0.5, V_term, np.ones(4, dtype=complex))
        I_load = np.conj(S_load / V_safe)
        I_inj = I_gen - I_load
        V_term = np.linalg.solve(Y_total, I_inj)
        
        I_out = (E_sys - V_term) * y_gen
        
        # Inverse Park
        Vd = np.zeros(4)
        Vq = np.zeros(4)
        Id = np.zeros(4)
        Iq = np.zeros(4)
        
        for i in range(4):
            delta = deltas[i]
            v_r, v_i = np.real(V_term[i]), np.imag(V_term[i])
            i_r, i_i = np.real(I_out[i]), np.imag(I_out[i])
            
            Vd[i] = v_r * np.cos(delta) + v_i * np.sin(delta)
            Vq[i] = -v_r * np.sin(delta) + v_i * np.cos(delta)
            Id[i] = i_r * np.cos(delta) + i_i * np.sin(delta)
            Iq[i] = -i_r * np.sin(delta) + i_i * np.cos(delta)
        
        return Id, Iq, Vd, Vq
    
    def dynamics(self, t, x_flat):
        """System dynamics using PyPHS gradients"""
        x = x_flat.reshape(self.n_gen, self.states_per_machine)
        
        gen_states = x[:, :7]
        exc_states = x[:, 7:11]
        gov_states = x[:, 11:13]
        
        mode = 'fault' if self.fault_start <= t < self.fault_end else 'normal'
        Id, Iq, Vd, Vq = self._solve_network(gen_states, mode)
        
        dxdt = np.zeros((self.n_gen, self.states_per_machine))
        
        for i in range(self.n_gen):
            gen_core = self.builder.generators[i]
            exc_core = self.builder.exciters[i]
            gov_core = self.builder.governors[i]
            
            meta = self.builder.gen_metadata[i]
            M = meta['M']
            D = meta['D']
            ra = meta['ra']
            omega_b = meta['omega_b']
            
            # Generator dynamics using PyPHS structure
            delta, p, psi_d, psi_q, psi_f, psi_kd, psi_kq = gen_states[i]
            omega = p / M
            
            # Get Tm from governor
            x1, x2 = gov_states[i]
            gov_meta = self.builder.gov_metadata[i]
            T2 = gov_meta['T2']
            T3 = gov_meta['T3']
            Dt = gov_meta['Dt']
            Tm = (T2/T3) * (x1 - x2) + x2 - Dt * (omega - 1.0)
            
            # Electrical torque
            Te = Vd[i] * Id[i] + Vq[i] * Iq[i]
            
            # Generator derivatives
            dxdt[i, 0] = omega_b * (omega - 1.0)
            dxdt[i, 1] = Tm - Te - D * (omega - 1.0)
            dxdt[i, 2] = Vd[i] - ra * Id[i] + omega_b * omega * psi_q
            dxdt[i, 3] = Vq[i] - ra * Iq[i] - omega_b * omega * psi_d
            
            # Field/dampers - get time constants from core.subs
            Td10 = [v for k, v in gen_core.subs.items() if 'Td10' in str(k)][0]
            Td20 = [v for k, v in gen_core.subs.items() if 'Td20' in str(k)][0]
            Tq20 = [v for k, v in gen_core.subs.items() if 'Tq20' in str(k)][0]
            
            Efd = exc_states[i, 1]  # vr1 is Efd
            dxdt[i, 4] = (Efd - psi_f) / Td10
            dxdt[i, 5] = -psi_kd / Td20
            dxdt[i, 6] = -psi_kq / Tq20
            
            # Exciter dynamics
            Vt = np.sqrt(Vd[i]**2 + Vq[i]**2)
            vm, vr1, vr2, vf = exc_states[i]
            
            TR = [v for k, v in exc_core.subs.items() if 'TR' in str(k)][0]
            TA = [v for k, v in exc_core.subs.items() if 'TA' in str(k)][0]
            KA = [v for k, v in exc_core.subs.items() if 'KA' in str(k)][0]
            Vref = [v for k, v in exc_core.subs.items() if 'Vref' in str(k)][0]
            
            # Get limits from metadata
            exc_meta = self.builder.exc_metadata[i]
            VRMAX = exc_meta.get('VRMAX', 5.2)
            VRMIN = exc_meta.get('VRMIN', -4.16)
            
            dxdt[i, 7] = (Vt - vm) / TR
            
            # Amplifier with saturation
            Verr = Vref - vm
            vr1_unlimited = (KA * Verr - vr1) / TA
            
            # Apply limits to derivative (anti-windup)
            if vr1 >= VRMAX and vr1_unlimited > 0:
                dxdt[i, 8] = 0.0
            elif vr1 <= VRMIN and vr1_unlimited < 0:
                dxdt[i, 8] = 0.0
            else:
                dxdt[i, 8] = vr1_unlimited
            
            dxdt[i, 9] = 0.0
            dxdt[i, 10] = 0.0
            
            # Governor dynamics (CRITICAL: 10x scaling to match ANDES)
            R = gov_meta['R']
            T1 = gov_meta['T1']
            T2 = gov_meta['T2']
            T3 = gov_meta['T3']
            wref = gov_meta['wref']
            Pref = gov_meta['Pref']
            VMAX = gov_meta.get('VMAX', 10.0)
            VMIN = gov_meta.get('VMIN', 0.0)
            
            # Droop control
            gate_cmd = Pref + (wref - omega) / R
            gate_limited = np.clip(gate_cmd, VMIN, VMAX)
            
            # Lead-lag with 10x multiplier
            dxdt[i, 11] = 10.0 * (gate_limited - x1) / T1
            dxdt[i, 12] = 10.0 * (x1 - x2) / T3
        
        return dxdt.flatten()
    
    def initialize_equilibrium(self):
        """Find equilibrium"""
        print("Finding equilibrium...")
        
        P_targets = np.array([7.459, 7.0, 7.0, 7.0])
        delta_init = np.array([0.08, 0.07, 0.06, 0.07])
        psi_f_init = np.array([1.3, 1.3, 1.3, 1.3])
        
        for iteration in range(50):
            # Build full temporary states with proper p initialization
            gen_states = np.zeros((4, 7))
            for i in range(4):
                M = self.builder.gen_metadata[i]['M']
                gen_states[i] = [delta_init[i], M * 1.0, 0.0, 0.0, psi_f_init[i], 0.0, 0.0]
            
            Id, Iq, Vd, Vq = self._solve_network(gen_states, 'normal')
            P_calc = Vd * Id + Vq * Iq
            V_mag = np.sqrt(Vd**2 + Vq**2)
            
            # Update with conservative steps
            max_error = 0
            for i in range(4):
                P_error = P_targets[i] - P_calc[i]
                delta_init[i] += 0.02 * P_error
                # Don't clip delta - need different angles for power flow!
                
                V_error = 1.0 - V_mag[i]
                psi_f_init[i] += 0.1 * V_error
                psi_f_init[i] = np.clip(psi_f_init[i], 0.5, 2.0)
                
                max_error = max(max_error, abs(P_error))
            
            if iteration % 10 == 0:
                print(f"  Iter {iteration}: P_error={max_error:.5f}, δ={np.degrees(delta_init)}")
            
            if max_error < 1e-4:
                print(f"  Converged in {iteration+1} iterations")
                break
        
        # Build full state
        x0 = np.zeros((4, self.states_per_machine))
        
        for i in range(4):
            meta = self.builder.gen_metadata[i]
            omega_b = meta['omega_b']
            M = meta['M']
            ra = meta['ra']
            
            # Flux linkages from steady-state (omega=1.0)
            psi_q = (-Vd[i] - ra * Id[i]) / omega_b
            psi_d = (Vq[i] + ra * Iq[i]) / omega_b
            
            # Generator states
            x0[i, 0] = delta_init[i]
            x0[i, 1] = M * 1.0
            x0[i, 2] = psi_d
            x0[i, 3] = psi_q
            x0[i, 4] = psi_f_init[i]
            x0[i, 5] = 0.0
            x0[i, 6] = 0.0
            
            # Exciter
            x0[i, 7] = V_mag[i]
            x0[i, 8] = psi_f_init[i]
            x0[i, 9] = 0.0
            x0[i, 10] = 0.0
            
            # Governor - CRITICAL: set to equilibrium power
            P_eq = P_calc[i]
            x0[i, 11] = P_eq
            x0[i, 12] = P_eq
            self.builder.gov_metadata[i]['Pref'] = P_eq
        
        print("Equilibrium:")
        for i in range(4):
            print(f"  Gen {i}: δ={np.degrees(delta_init[i]):.2f}°, P={P_calc[i]:.3f}, V={V_mag[i]:.4f}")
        
        return x0
    
    def simulate(self, x0, t_end=15.0, n_points=1500):
        """Run simulation"""
        print(f"\nSimulating {t_end}s, fault @ t={self.fault_start}-{self.fault_end}s")
        
        last_print = [0]
        def monitor(t, x):
            if t - last_print[0] > 0.5:
                x_r = x.reshape(self.n_gen, self.states_per_machine)
                omegas = [x_r[i, 1] / self.builder.gen_metadata[i]['M'] for i in range(4)]
                omega_avg = np.mean(omegas)
                delta_max = np.max(np.abs(np.degrees(x_r[:, 0])))
                status = "FAULT" if self.fault_start <= t < self.fault_end else "normal"
                print(f"  t={t:.2f}s [{status:6s}]: ω={omega_avg:.6f}, δ_max={delta_max:.1f}°")
                last_print[0] = t
            return 1
        monitor.terminal = False
        
        sol = solve_ivp(
            self.dynamics,
            (0, t_end),
            x0.flatten(),
            t_eval=np.linspace(0, t_end, n_points),
            method='RK45',
            rtol=1e-6,
            atol=1e-8,
            events=monitor
        )
        
        if sol.success:
            print("Success!")
        return sol
    
    def plot_results(self, sol):
        """Plot results"""
        x_hist = sol.y.T.reshape(-1, self.n_gen, self.states_per_machine)
        t = sol.t
        
        fig, axes = plt.subplots(2, 3, figsize=(15, 8))
        colors = ['b', 'r', 'g', 'm']
        
        for ax_row in axes:
            for ax in ax_row:
                ax.axvspan(self.fault_start, self.fault_end, alpha=0.2, color='red')
        
        for g in range(4):
            delta = np.degrees(x_hist[:, g, 0])
            omega = x_hist[:, g, 1] / self.builder.gen_metadata[g]['M']
            Efd = x_hist[:, g, 8]
            x1 = x_hist[:, g, 11]
            x2 = x_hist[:, g, 12]
            
            # Calculate Tm from governor states
            gov_meta = self.builder.gov_metadata[g]
            T2 = gov_meta['T2']
            T3 = gov_meta['T3']
            Dt = gov_meta['Dt']
            Tm = (T2/T3) * (x1 - x2) + x2 - Dt * (omega - 1.0)
            
            axes[0, 0].plot(t, delta, colors[g], label=f'Gen {g+1}', linewidth=1.5)
            axes[0, 1].plot(t, omega, colors[g], linewidth=1.5)
            axes[0, 2].plot(t, Efd, colors[g], linewidth=1.5)
            axes[1, 0].plot(t, Tm, colors[g], linewidth=1.5)
            axes[1, 1].plot(t, (omega - 1.0) * 60, colors[g], linewidth=1.5)
            axes[1, 2].plot(t, delta - delta[0], colors[g], linewidth=1.5)
        
        axes[0, 0].set_ylabel('Angle (deg)')
        axes[0, 0].legend()
        axes[0, 0].grid(True)
        axes[0, 0].set_title('Rotor Angles')
        
        axes[0, 1].set_ylabel('Speed (pu)')
        axes[0, 1].grid(True)
        axes[0, 1].set_title('Rotor Speeds')
        
        axes[0, 2].set_ylabel('Efd (pu)')
        axes[0, 2].grid(True)
        axes[0, 2].set_title('Exciter Output')
        
        axes[1, 0].set_ylabel('Tm (pu)')
        axes[1, 0].set_xlabel('Time (s)')
        axes[1, 0].grid(True)
        axes[1, 0].set_title('Mechanical Torque (Governor)')
        
        axes[1, 1].set_ylabel('Δf (Hz)')
        axes[1, 1].set_xlabel('Time (s)')
        axes[1, 1].grid(True)
        axes[1, 1].set_title('Frequency Deviation')
        
        axes[1, 2].set_ylabel('Δδ (deg)')
        axes[1, 2].set_xlabel('Time (s)')
        axes[1, 2].grid(True)
        axes[1, 2].set_title('Angle Deviation')
        
        plt.tight_layout()
        
        output_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'outputs')
        os.makedirs(output_dir, exist_ok=True)
        output_path = os.path.join(output_dir, 'fault_simulation.png')
        plt.savefig(output_path, dpi=150)
        print(f"\nPlot saved: {output_path}")
        plt.show()