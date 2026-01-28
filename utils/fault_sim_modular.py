"""
Modular Fault Simulator - Uses system_builder + coordinator
Fully dynamic - works with any number of generators
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

        # Base frequency
        self.omega_b = 2 * np.pi * 60

        # Fault configuration (can be modified)
        self.fault_enabled = True
        self.fault_bus = 2
        self.fault_impedance = 0.005j
        self.fault_start = 1.0
        self.fault_duration = 0.1

        # Extract initial power flow data from JSON
        self._extract_initial_conditions()

        print(f"\nModular Fault Simulator: {self.n_gen} generators, {self.total_states} states")

    def _extract_initial_conditions(self):
        """Extract initial power and voltage values from JSON data"""
        system_data = self.builder.system_data

        # Get generator power targets and initial angles from JSON
        self.P_targets = np.zeros(self.n_gen)
        self.Q_targets = np.zeros(self.n_gen)
        self.V_targets = np.zeros(self.n_gen)
        self.delta_init = np.zeros(self.n_gen)

        # Map generator index to bus
        gen_to_bus = self.coordinator.gen_to_bus

        # Get bus data for initial angles and voltages
        bus_data = {b['idx']: b for b in system_data.get('Bus', [])}

        # Get power from Slack and PV data
        slack_data = {s['bus']: s for s in system_data.get('Slack', [])}
        pv_data = {p['bus']: p for p in system_data.get('PV', [])}

        for i in range(self.n_gen):
            bus_idx = gen_to_bus[i]

            # Get initial voltage and angle from bus data
            if bus_idx in bus_data:
                self.V_targets[i] = bus_data[bus_idx].get('v0', 1.0)
                self.delta_init[i] = bus_data[bus_idx].get('a0', 0.5)
            else:
                self.V_targets[i] = 1.0
                self.delta_init[i] = 0.5

            # Get power from Slack or PV
            if bus_idx in slack_data:
                self.P_targets[i] = slack_data[bus_idx].get('p0', 7.0)
                self.Q_targets[i] = slack_data[bus_idx].get('q0', 0.0)
            elif bus_idx in pv_data:
                self.P_targets[i] = pv_data[bus_idx].get('p0', 7.0)
                self.Q_targets[i] = pv_data[bus_idx].get('q0', 0.0)
            else:
                # Fallback
                self.P_targets[i] = 7.0
                self.Q_targets[i] = 0.0

        print(f"Initial conditions from JSON:")
        for i in range(self.n_gen):
            print(f"  Gen {i}: P={self.P_targets[i]:.3f}, V={self.V_targets[i]:.4f}, delta={np.degrees(self.delta_init[i]):.2f} deg")

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

        # Determine if fault is active
        use_fault = self.fault_enabled and self.fault_start <= t < (self.fault_start + self.fault_duration)

        # Solve network
        Id, Iq, Vd, Vq = self.coordinator.solve_network(
            gen_states,
            use_fault=use_fault,
            fault_bus=self.fault_bus,
            fault_impedance=self.fault_impedance
        )

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

            # Generator electrical torque
            Te = Vd[i] * Id[i] + Vq[i] * Iq[i]

            # Swing equations
            dxdt[i, 0] = omega_b * (omega - 1.0)  # d(delta)/dt
            dxdt[i, 1] = Tm - Te - D * (omega - 1.0)  # d(p)/dt = M * d(omega)/dt

            # Flux linkage equations (stator)
            dxdt[i, 2] = Vd[i] - ra * Id[i] + omega_b * omega * psi_q  # d(psi_d)/dt
            dxdt[i, 3] = Vq[i] - ra * Iq[i] - omega_b * omega * psi_d  # d(psi_q)/dt

            # Rotor flux dynamics - extract time constants from generator
            Td10 = self._get_param(gen_core, 'Td10', 8.0)
            Td20 = self._get_param(gen_core, 'Td20', 0.03)
            Tq20 = self._get_param(gen_core, 'Tq20', 0.05)

            Efd = exc_states[i, 1]  # Exciter output
            dxdt[i, 4] = (Efd - psi_f) / Td10  # d(psi_f)/dt
            dxdt[i, 5] = -psi_kd / Td20  # d(psi_kd)/dt
            dxdt[i, 6] = -psi_kq / Tq20  # d(psi_kq)/dt

            # Exciter dynamics
            Vt = np.sqrt(Vd[i]**2 + Vq[i]**2)
            vm, vr1, vr2, vf = exc_states[i]

            TR = self._get_param(exc_core, 'TR', 0.02)
            TA = self._get_param(exc_core, 'TA', 0.02)
            KA = self._get_param(exc_core, 'KA', 20.0)
            Vref = self._get_param(exc_core, 'Vref', 1.0)

            exc_meta = self.builder.exc_metadata[i]
            VRMAX = exc_meta.get('VRMAX', 5.2)
            VRMIN = exc_meta.get('VRMIN', -4.16)

            dxdt[i, 7] = (Vt - vm) / TR  # d(vm)/dt - voltage measurement

            Verr = Vref - vm
            vr1_unlimited = (KA * Verr - vr1) / TA

            # Anti-windup limiter
            if vr1 >= VRMAX and vr1_unlimited > 0:
                dxdt[i, 8] = 0.0
            elif vr1 <= VRMIN and vr1_unlimited < 0:
                dxdt[i, 8] = 0.0
            else:
                dxdt[i, 8] = vr1_unlimited

            dxdt[i, 9] = 0.0  # vr2 (not used in simplified model)
            dxdt[i, 10] = 0.0  # vf (not used in simplified model)

            # Governor dynamics
            gate_cmd = gov_meta['Pref'] + (gov_meta['wref'] - omega) / gov_meta['R']
            gate_limited = np.clip(gate_cmd, gov_meta.get('VMIN', 0.0), gov_meta.get('VMAX', 10.0))

            dxdt[i, 11] = 10.0 * (gate_limited - x1) / gov_meta['T1']  # d(x1)/dt
            dxdt[i, 12] = 10.0 * (x1 - x2) / gov_meta['T3']  # d(x2)/dt

        return dxdt.flatten()

    def _get_param(self, core, name, default):
        """Extract parameter from component subs dictionary"""
        for k, v in core.subs.items():
            if name in str(k):
                return v
        return default

    def initialize_equilibrium(self):
        """
        Find equilibrium using power flow results from JSON.

        Uses direct calculation from power flow:
        - V, theta from bus data
        - P, Q from generator data
        - Calculate E'' and delta from: E'' = V + jXd'' * I
        """
        print("Finding equilibrium from power flow...")

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
                P_gen[i] = self.P_targets[i]
                Q_gen[i] = self.Q_targets[i]

            # Generator parameters (on system base)
            Xd2 = meta.get('xd2', 0.0278)  # Already on system base
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
            # E''_q = gd1 * psi_f (approximately, for GENROU)
            psi_f_vals[i] = E_mag / gd1

            # Transform V and I to machine (dq) frame
            # Inverse Park: X_d = X_R*sin(delta) - X_I*cos(delta)
            #               X_q = X_R*cos(delta) + X_I*sin(delta)
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
        print(f"Power verification:")
        for i in range(self.n_gen):
            print(f"  Gen {i}: P_target={P_gen[i]:.3f}, P_calc={P_check[i]:.3f}, diff={abs(P_gen[i]-P_check[i]):.5f}")

        # Build full state vector
        x0 = np.zeros((self.n_gen, self.states_per_machine))

        for i in range(self.n_gen):
            meta = self.builder.gen_metadata[i]
            omega_b = meta['omega_b']
            M = meta['M']
            ra = meta['ra']

            # Calculate flux linkages from terminal conditions
            # At steady state: psi_d = (Vq + ra*Iq) / omega_b
            #                  psi_q = -(Vd + ra*Id) / omega_b
            psi_d = (Vq[i] + ra * Iq[i]) / omega_b
            psi_q = -(Vd[i] + ra * Id[i]) / omega_b

            # Generator states
            x0[i, 0] = delta_vals[i]  # delta
            x0[i, 1] = M * 1.0  # p = M * omega (omega=1.0 at equilibrium)
            x0[i, 2] = psi_d  # psi_d
            x0[i, 3] = psi_q  # psi_q
            x0[i, 4] = psi_f_vals[i]  # psi_f (field flux)
            x0[i, 5] = 0.0  # psi_kd (d-axis damper)
            x0[i, 6] = 0.0  # psi_kq (q-axis damper)

            # Exciter states
            x0[i, 7] = V_mag[i]  # vm (voltage measurement)
            x0[i, 8] = psi_f_vals[i]  # vr1 (exciter output = Efd at equilibrium)
            x0[i, 9] = 0.0  # vr2
            x0[i, 10] = 0.0  # vf

            # Governor states - Pref is set for non-slack generators
            # For slack (Gen 0), we still need Pref for governor but it will adjust
            x0[i, 11] = P_gen[i]  # x1 (governor state)
            x0[i, 12] = P_gen[i]  # x2 (governor state)
            self.builder.gov_metadata[i]['Pref'] = P_gen[i]

            # Update Vref to maintain equilibrium Efd
            # At equilibrium: d(Efd)/dt = 0 => KA*(Vref - Vt) = Efd
            # So: Vref = Vt + Efd/KA
            exc_core = self.builder.exciters[i]
            KA = self._get_param(exc_core, 'KA', 200.0)
            Efd_eq = x0[i, 8]
            Vref_correct = V_mag[i] + Efd_eq / KA

            for key in exc_core.subs.keys():
                if 'Vref' in str(key):
                    exc_core.subs[key] = Vref_correct
                    break

        print("\nEquilibrium:")
        for i in range(self.n_gen):
            print(f"  Gen {i}: delta={np.degrees(delta_vals[i]):.2f} deg, P={P_gen[i]:.3f} pu, V={V_mag[i]:.4f} pu, psi_f={psi_f_vals[i]:.3f}")

        # Verify that solve_network returns consistent values
        print("\nVerifying solve_network consistency...")
        gen_states = x0[:, :7]
        Id_net, Iq_net, Vd_net, Vq_net = self.coordinator.solve_network(gen_states)
        P_net = Vd_net * Id_net + Vq_net * Iq_net
        V_net = np.sqrt(Vd_net**2 + Vq_net**2)

        for i in range(self.n_gen):
            print(f"  Gen {i}: P_init={P_gen[i]:.3f}, P_net={P_net[i]:.3f}, diff={P_gen[i]-P_net[i]:.4f}")
            print(f"          Vd_init={Vd[i]:.4f}, Vd_net={Vd_net[i]:.4f}")
            print(f"          Vq_init={Vq[i]:.4f}, Vq_net={Vq_net[i]:.4f}")

        return x0

    def simulate(self, x0, t_end=15.0, n_points=1500):
        """Run simulation"""
        fault_msg = f"fault @ Bus {self.fault_bus}, t={self.fault_start}-{self.fault_start + self.fault_duration}s" if self.fault_enabled else "no fault"
        print(f"\nSimulating {t_end}s, {fault_msg}")

        last_print = [0]
        def monitor(t, x):
            if t - last_print[0] > 0.5:
                x_r = x.reshape(self.n_gen, self.states_per_machine)
                omegas = [x_r[i, 1] / self.builder.gen_metadata[i]['M'] for i in range(self.n_gen)]
                omega_avg = np.mean(omegas)
                delta_max = np.max(np.abs(np.degrees(x_r[:, 0])))

                if self.fault_enabled:
                    status = "FAULT" if self.fault_start <= t < (self.fault_start + self.fault_duration) else "normal"
                else:
                    status = "normal"

                print(f"  t={t:.2f}s [{status:6s}]: omega={omega_avg:.6f}, delta_max={delta_max:.1f} deg")
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
        colors = ['b', 'r', 'g', 'm', 'c', 'y', 'k']  # Support more generators

        if self.fault_enabled:
            for ax_row in axes:
                for ax in ax_row:
                    ax.axvspan(self.fault_start, self.fault_start + self.fault_duration, alpha=0.2, color='red')

        for g in range(self.n_gen):
            delta = np.degrees(x_hist[:, g, 0])
            omega = x_hist[:, g, 1] / self.builder.gen_metadata[g]['M']
            Efd = x_hist[:, g, 8]
            x1 = x_hist[:, g, 11]
            x2 = x_hist[:, g, 12]

            gov_meta = self.builder.gov_metadata[g]
            Tm = (gov_meta['T2']/gov_meta['T3']) * (x1 - x2) + x2 - gov_meta['Dt'] * (omega - 1.0)

            color = colors[g % len(colors)]
            axes[0, 0].plot(t, delta, color, label=f'Gen {g+1}', linewidth=1.5)
            axes[0, 1].plot(t, omega, color, linewidth=1.5)
            axes[0, 2].plot(t, Efd, color, linewidth=1.5)
            axes[1, 0].plot(t, Tm, color, linewidth=1.5)
            axes[1, 1].plot(t, (omega - 1.0) * 60, color, linewidth=1.5)
            axes[1, 2].plot(t, delta - delta[0], color, linewidth=1.5)

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

        axes[1, 1].set_ylabel('Freq Dev (Hz)')
        axes[1, 1].set_xlabel('Time (s)')
        axes[1, 1].grid(True)
        axes[1, 1].set_title('Frequency Deviation')

        axes[1, 2].set_ylabel('Angle Dev (deg)')
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
