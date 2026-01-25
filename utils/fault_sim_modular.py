"""
Modular Fault Simulator - Uses system_builder + coordinator
"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from utils.system_builder import PowerSystemBuilder
from utils.system_coordinator import PowerSystemCoordinator


class ModularFaultSimulator:
    """Fault simulator using modular architecture"""
    
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
        
        # Fault configuration (can be modified)
        self.fault_enabled = True
        self.fault_bus = 2
        self.fault_impedance = 0.005j
        self.fault_start = 1.0
        self.fault_duration = 0.1
        
        print(f"\nModular Fault Simulator: {self.total_states} states")
    
    def set_fault(self, bus_idx, impedance, start_time, duration):
        """Configure fault parameters"""
        self.fault_bus = bus_idx
        self.fault_impedance = impedance
        self.fault_start = start_time
        self.fault_duration = duration
        self.fault_enabled = True
    
    def disable_fault(self):
        """Disable fault"""
        self.fault_enabled = False
    
    def dynamics(self, t, x_flat):
        """System dynamics"""
        x = x_flat.reshape(self.n_gen, self.states_per_machine)
        
        gen_states = x[:, :7]
        exc_states = x[:, 7:11]
        gov_states = x[:, 11:13]
        
        # Determine Ybus based on fault status
        if self.fault_enabled and self.fault_start <= t < (self.fault_start + self.fault_duration):
            Ybus = self.coordinator.apply_fault(self.fault_bus, self.fault_impedance)
        else:
            Ybus = None  # Use base
        
        # Solve network
        Id, Iq, Vd, Vq = self.coordinator.solve_network(gen_states, Ybus)
        
        # Component dynamics
        dxdt = np.zeros((self.n_gen, self.states_per_machine))
        
        for i in range(self.n_gen):
            meta = self.builder.gen_metadata[i]
            gen_core = self.builder.generators[i]
            exc_core = self.builder.exciters[i]
            
            M = meta['M']
            D = meta['D']
            ra = meta['ra']
            omega_b = meta['omega_b']
            
            delta, p, psi_d, psi_q, psi_f, psi_kd, psi_kq = gen_states[i]
            omega = p / M
            
            # Governor
            x1, x2 = gov_states[i]
            gov_meta = self.builder.gov_metadata[i]
            Tm = (gov_meta['T2']/gov_meta['T3']) * (x1 - x2) + x2 - gov_meta['Dt'] * (omega - 1.0)
            
            # Generator
            Te = Vd[i] * Id[i] + Vq[i] * Iq[i]
            dxdt[i, 0] = omega_b * (omega - 1.0)
            dxdt[i, 1] = Tm - Te - D * (omega - 1.0)
            dxdt[i, 2] = Vd[i] - ra * Id[i] + omega_b * omega * psi_q
            dxdt[i, 3] = Vq[i] - ra * Iq[i] - omega_b * omega * psi_d
            
            Td10 = [v for k, v in gen_core.subs.items() if 'Td10' in str(k)][0]
            Td20 = [v for k, v in gen_core.subs.items() if 'Td20' in str(k)][0]
            Tq20 = [v for k, v in gen_core.subs.items() if 'Tq20' in str(k)][0]
            
            Efd = exc_states[i, 1]
            dxdt[i, 4] = (Efd - psi_f) / Td10
            dxdt[i, 5] = -psi_kd / Td20
            dxdt[i, 6] = -psi_kq / Tq20
            
            # Exciter
            Vt = np.sqrt(Vd[i]**2 + Vq[i]**2)
            vm, vr1, vr2, vf = exc_states[i]
            
            TR = [v for k, v in exc_core.subs.items() if 'TR' in str(k)][0]
            TA = [v for k, v in exc_core.subs.items() if 'TA' in str(k)][0]
            KA = [v for k, v in exc_core.subs.items() if 'KA' in str(k)][0]
            Vref = [v for k, v in exc_core.subs.items() if 'Vref' in str(k)][0]
            
            exc_meta = self.builder.exc_metadata[i]
            VRMAX = exc_meta.get('VRMAX', 5.2)
            VRMIN = exc_meta.get('VRMIN', -4.16)
            
            dxdt[i, 7] = (Vt - vm) / TR
            
            Verr = Vref - vm
            vr1_unlimited = (KA * Verr - vr1) / TA
            
            if vr1 >= VRMAX and vr1_unlimited > 0:
                dxdt[i, 8] = 0.0
            elif vr1 <= VRMIN and vr1_unlimited < 0:
                dxdt[i, 8] = 0.0
            else:
                dxdt[i, 8] = vr1_unlimited
            
            dxdt[i, 9] = 0.0
            dxdt[i, 10] = 0.0
            
            # Governor
            gate_cmd = gov_meta['Pref'] + (gov_meta['wref'] - omega) / gov_meta['R']
            gate_limited = np.clip(gate_cmd, gov_meta.get('VMIN', 0.0), gov_meta.get('VMAX', 10.0))
            
            dxdt[i, 11] = 10.0 * (gate_limited - x1) / gov_meta['T1']
            dxdt[i, 12] = 10.0 * (x1 - x2) / gov_meta['T3']
        
        return dxdt.flatten()
    
    def initialize_equilibrium(self):
        """Find equilibrium"""
        print("Finding equilibrium...")
        
        P_targets = np.array([7.459, 7.0, 7.0, 7.0])
        delta_init = np.array([0.08, 0.07, 0.06, 0.07])
        psi_f_init = np.array([1.3, 1.3, 1.3, 1.3])
        
        for iteration in range(50):
            gen_states = np.zeros((4, 7))
            for i in range(4):
                M = self.builder.gen_metadata[i]['M']
                gen_states[i] = [delta_init[i], M * 1.0, 0.0, 0.0, psi_f_init[i], 0.0, 0.0]
            
            Id, Iq, Vd, Vq = self.coordinator.solve_network(gen_states)
            P_calc = Vd * Id + Vq * Iq
            V_mag = np.sqrt(Vd**2 + Vq**2)
            
            max_error = 0
            for i in range(4):
                P_error = P_targets[i] - P_calc[i]
                delta_init[i] += 0.02 * P_error
                
                V_error = 1.0 - V_mag[i]
                psi_f_init[i] += 0.1 * V_error
                psi_f_init[i] = np.clip(psi_f_init[i], 0.5, 2.0)
                
                max_error = max(max_error, abs(P_error))
            
            if iteration % 10 == 0:
                print(f"  Iter {iteration}: P_error={max_error:.5f}")
            
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
            
            psi_q = (-Vd[i] - ra * Id[i]) / omega_b
            psi_d = (Vq[i] + ra * Iq[i]) / omega_b
            
            x0[i, 0] = delta_init[i]
            x0[i, 1] = M * 1.0
            x0[i, 2] = psi_d
            x0[i, 3] = psi_q
            x0[i, 4] = psi_f_init[i]
            x0[i, 5] = 0.0
            x0[i, 6] = 0.0
            
            x0[i, 7] = V_mag[i]
            x0[i, 8] = psi_f_init[i]
            x0[i, 9] = 0.0
            x0[i, 10] = 0.0
            
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
        fault_msg = f"fault @ Gen {self.fault_bus}, t={self.fault_start}-{self.fault_start + self.fault_duration}s" if self.fault_enabled else "no fault"
        print(f"\nSimulating {t_end}s, {fault_msg}")
        
        last_print = [0]
        def monitor(t, x):
            if t - last_print[0] > 0.5:
                x_r = x.reshape(self.n_gen, self.states_per_machine)
                omegas = [x_r[i, 1] / self.builder.gen_metadata[i]['M'] for i in range(4)]
                omega_avg = np.mean(omegas)
                delta_max = np.max(np.abs(np.degrees(x_r[:, 0])))
                
                if self.fault_enabled:
                    status = "FAULT" if self.fault_start <= t < (self.fault_start + self.fault_duration) else "normal"
                else:
                    status = "normal"
                    
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
    
    def plot_results(self, sol, filename='fault_simulation.png'):
        """Plot results"""
        x_hist = sol.y.T.reshape(-1, self.n_gen, self.states_per_machine)
        t = sol.t
        
        fig, axes = plt.subplots(2, 3, figsize=(15, 8))
        colors = ['b', 'r', 'g', 'm']
        
        if self.fault_enabled:
            for ax_row in axes:
                for ax in ax_row:
                    ax.axvspan(self.fault_start, self.fault_start + self.fault_duration, alpha=0.2, color='red')
        
        for g in range(4):
            delta = np.degrees(x_hist[:, g, 0])
            omega = x_hist[:, g, 1] / self.builder.gen_metadata[g]['M']
            Efd = x_hist[:, g, 8]
            x1 = x_hist[:, g, 11]
            x2 = x_hist[:, g, 12]
            
            gov_meta = self.builder.gov_metadata[g]
            Tm = (gov_meta['T2']/gov_meta['T3']) * (x1 - x2) + x2 - gov_meta['Dt'] * (omega - 1.0)
            
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
        axes[1, 0].set_title('Mechanical Torque')
        
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
        output_path = os.path.join(output_dir, filename)
        plt.savefig(output_path, dpi=150)
        print(f"\nPlot saved: {output_path}")
        plt.show()
