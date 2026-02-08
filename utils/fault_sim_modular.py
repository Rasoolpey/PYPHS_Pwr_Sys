"""
Modular Fault Simulator - Uses system_builder + coordinator
Fully dynamic - works with any number of generators
"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import json
import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from utils.system_builder import PowerSystemBuilder
from utils.system_coordinator import PowerSystemCoordinator
from utils.power_flow import run_power_flow


class ModularFaultSimulator:
    """Fault simulator using modular architecture"""

    def __init__(self, system_json=None, simulation_json=None, builder_or_json=None):
        """
        Initialize simulator with system and optional simulation config.
        
        Args:
            system_json: Path to system configuration (Bus, Line, GENROU, etc.)
            simulation_json: Path to simulation config (time, fault, solver, output)
                            If None, uses default values
            builder_or_json: (deprecated) PowerSystemBuilder or JSON filepath
                             For backward compatibility only
        """
        # Handle backward compatibility
        if builder_or_json is not None:
            if isinstance(builder_or_json, str):
                system_json = builder_or_json
            else:
                self.builder = builder_or_json
        
        # Build system from system JSON
        if system_json is not None:
            self.builder = PowerSystemBuilder(system_json)
            self.builder.build_all_components()
        
        # Store reference to system data for extracting initial conditions
        self.system_json_path = system_json

        # System coordinator handles network
        self.coordinator = PowerSystemCoordinator(self.builder)

        self.n_gen = len(self.builder.generators)
        self.n_grid = len(self.builder.grids)
        
        # Dynamically determine state counts from actual components using n_states attribute
        if len(self.builder.generators) > 0:
            self.n_gen_states = self.builder.generators[0].n_states
        else:
            self.n_gen_states = 7  # Fallback
        
        # Get exciter state count dynamically from first exciter
        if len(self.builder.exciters) > 0:
            self.n_exc_states = self.builder.exciters[0].n_states
        else:
            self.n_exc_states = 4  # Fallback
        
        # Get governor state count dynamically from first governor
        if len(self.builder.governors) > 0:
            self.n_gov_states = self.builder.governors[0].n_states
        else:
            self.n_gov_states = 2  # Fallback
        
        # Get grid state count dynamically
        if self.n_grid > 0:
            self.n_grid_states = self.builder.grids[0].n_states
        else:
            self.n_grid_states = 0
        
        self.states_per_machine = self.n_gen_states + self.n_exc_states + self.n_gov_states
        self.total_states = self.n_gen * self.states_per_machine + self.n_grid * self.n_grid_states

        # Base frequency
        self.omega_b = 2 * np.pi * 60

        # Load simulation configuration
        self._load_simulation_config(simulation_json)
        
        # Power flow solver (initialized but not run by default)
        self.power_flow_solved = False

        # Extract initial power flow data from JSON
        self._extract_initial_conditions()

        print(f"\nModular Fault Simulator: {self.n_gen} generators, {self.n_grid} grids, {self.total_states} states")
        print(f"  Component states: Gen={self.n_gen_states}, Exc={self.n_exc_states}, Gov={self.n_gov_states}, Grid={self.n_grid_states}")
        if len(self.builder.exciters) > 0:
            print(f"  Exciter type: {self.builder.exciters[0].model_name}")
        if len(self.builder.governors) > 0:
            print(f"  Governor type: {self.builder.governors[0].model_name}")
    
    def _load_simulation_config(self, simulation_json):
        """Load simulation configuration from JSON file or use defaults"""
        if simulation_json:
            print(f"Loading simulation config from: {simulation_json}")
            with open(simulation_json, 'r') as f:
                sim_config = json.load(f)
        else:
            print("Using default simulation configuration")
            sim_config = {}
        
        # Load time parameters
        time_config = sim_config.get('time', {})
        self.t_end = time_config.get('t_end', 15.0)
        self.n_points = time_config.get('n_points', 3000)
        
        # Load fault config
        fault_config = sim_config.get('fault', {})
        self.fault_enabled = fault_config.get('enabled', True)
        if self.fault_enabled:
            self.fault_bus = fault_config.get('bus', 2)
            impedance_real = fault_config.get('impedance_real', 0.0)
            impedance_imag = fault_config.get('impedance_imag', 0.005)
            self.fault_impedance = complex(impedance_real, impedance_imag)
            self.fault_start = fault_config.get('start_time', 1.0)
            self.fault_duration = fault_config.get('duration', 0.1)
        else:
            # Set default values even when fault is disabled
            self.fault_bus = 2
            self.fault_impedance = 0.005j
            self.fault_start = 1.0
            self.fault_duration = 0.1
        
        # Load solver config
        solver_config = sim_config.get('solver', {})
        self.solver_method = solver_config.get('method', 'Radau')
        self.rtol = solver_config.get('rtol', 1e-6)
        self.atol = solver_config.get('atol', 1e-8)
        
        # Load output config
        output_config = sim_config.get('output', {})
        self.output_dir = output_config.get('directory', 'outputs')
        self.save_states = output_config.get('save_states', True)
        self.plot_format = output_config.get('plot_format', 'png')
        
        # Load initialization config
        init_config = sim_config.get('initialization', {})
        self.auto_power_flow = init_config.get('run_power_flow', False)
        self.power_flow_verbose = init_config.get('power_flow_verbose', True)
        
        print(f"  Simulation time: t_end={self.t_end}s, n_points={self.n_points}")
        print(f"  Fault: enabled={self.fault_enabled}, bus={self.fault_bus}, Z={self.fault_impedance}")
        print(f"  Solver: {self.solver_method}, rtol={self.rtol}, atol={self.atol}")
        print(f"  Output: dir={self.output_dir}, format={self.plot_format}")
        print(f"  Initialization: auto_power_flow={self.auto_power_flow}")

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
    
    def solve_power_flow(self, verbose=True):
        """
        Solve AC power flow to find accurate initial conditions.
        
        This should be called BEFORE initialize_equilibrium() to get
        consistent voltage magnitudes, angles, and power injections.
        
        Args:
            verbose: Print power flow results
        
        Returns:
            converged (bool): True if power flow converged
        """
        print("\n" + "="*70)
        print("   POWER FLOW INITIALIZATION")
        print("="*70)
        
        pf_solver = run_power_flow(self.builder, self.coordinator, verbose=verbose)
        
        if pf_solver is not None:
            self.power_flow_solved = True
            # Re-extract initial conditions from updated system data
            self._extract_initial_conditions()
            return True
        else:
            self.power_flow_solved = False
            return False

    def dynamics(self, t, x_flat):
        """System dynamics"""
        # Split state vector into generator and grid parts
        gen_size = self.n_gen * self.states_per_machine
        x_gen_flat = x_flat[:gen_size]
        x_grid_flat = x_flat[gen_size:]
        
        x = x_gen_flat.reshape(self.n_gen, self.states_per_machine)
        x_grid = x_grid_flat.reshape(self.n_grid, self.n_grid_states) if self.n_grid > 0 else np.array([])

        # Dynamically extract states based on actual component sizes
        gen_states = x[:, :self.n_gen_states]
        exc_offset = self.n_gen_states
        exc_states = x[:, exc_offset:exc_offset+self.n_exc_states]
        gov_offset = exc_offset + self.n_exc_states
        gov_states = x[:, gov_offset:gov_offset+self.n_gov_states]

        # Determine if fault is active
        use_fault = self.fault_enabled and self.fault_start <= t < (self.fault_start + self.fault_duration)

        # Extract grid voltages if grids exist
        grid_voltages = None
        if self.n_grid > 0:
            # Grid voltages are complex phasors: V = V_mag * exp(j*theta)
            grid_voltages = np.array([x_grid[i, 0] * np.exp(1j * x_grid[i, 1]) 
                                      for i in range(self.n_grid)])

        # Solve network with grid voltages as boundary conditions
        Id, Iq, Vd, Vq = self.coordinator.solve_network(
            gen_states,
            grid_voltages=grid_voltages,
            use_fault=use_fault,
            fault_bus=self.fault_bus,
            fault_impedance=self.fault_impedance
        )

        # Component dynamics - Call each component's own dynamics function!
        dxdt = np.zeros((self.n_gen, self.states_per_machine))

        for i in range(self.n_gen):
            # Get component cores and metadata
            gen_core = self.builder.generators[i]
            exc_core = self.builder.exciters[i]
            gov_core = self.builder.governors[i]
            
            gen_meta = self.builder.gen_metadata[i]
            exc_meta = self.builder.exc_metadata[i]
            gov_meta = self.builder.gov_metadata[i]

            # Extract states for each component
            gen_x = gen_states[i]
            exc_x = exc_states[i]
            gov_x = gov_states[i]
            
            # Calculate auxiliary quantities needed for ports
            delta, p = gen_x[0], gen_x[1]
            M = gen_meta['M']
            omega = p / M
            Vt = np.sqrt(Vd[i]**2 + Vq[i]**2)
            
            # Get Efd from exciter using output function (generic for all exciter types)
            exc_ports_for_output = {
                'Vt': Vt, 'Vd': Vd[i], 'Vq': Vq[i],
                'Id': Id[i], 'Iq': Iq[i],
                'XadIfd': gen_x[4]  # psi_f
            }
            if exc_core.output_fn is not None:
                Efd = exc_core.output_fn(exc_x, exc_ports_for_output, exc_meta)
            else:
                # Fallback: assume Efd is the second state (works for most exciters)
                Efd = exc_x[min(1, len(exc_x)-1)] if len(exc_x) > 1 else 1.0
            
            # Governor: Get Tm from output function (generic for all governor types)
            gov_ports = {'omega': omega}
            if gov_core.output_fn is not None:
                Tm = gov_core.output_fn(gov_x, gov_ports, gov_meta)
            else:
                # Fallback: assume Tm is the last state (works for most governors)
                Tm = gov_x[-1] if len(gov_x) > 0 else 1.0
            
            # Governor dynamics - use the private _dynamics_fn
            gov_dxdt = gov_core._dynamics_fn(gov_x, gov_ports, gov_meta)
            
            # Generator: call its dynamics function
            gen_ports = {
                'Id': Id[i], 'Iq': Iq[i],
                'Vd': Vd[i], 'Vq': Vq[i],
                'Tm': Tm, 'Efd': Efd
            }
            gen_dxdt = gen_core._dynamics_fn(gen_x, gen_ports, gen_meta)
            
            # Exciter: call its dynamics function
            exc_ports = {
                'Vt': Vt, 'Vd': Vd[i], 'Vq': Vq[i],
                'Id': Id[i], 'Iq': Iq[i],
                'XadIfd': gen_x[4],  # psi_f
                'Efd_fb': Efd
            }
            exc_dxdt = exc_core._dynamics_fn(exc_x, exc_ports, exc_meta)
            
            # Assemble into full derivative vector
            dxdt[i, :self.n_gen_states] = gen_dxdt
            dxdt[i, self.n_gen_states:self.n_gen_states+self.n_exc_states] = exc_dxdt
            dxdt[i, self.n_gen_states+self.n_exc_states:] = gov_dxdt

        # Grid dynamics - maintain constant voltage
        dxdt_grid = np.zeros((self.n_grid, self.n_grid_states))
        for i in range(self.n_grid):
            grid_core = self.builder.grids[i]
            grid_meta = self.builder.grid_metadata[i]
            grid_x = x_grid[i]
            
            # Grid doesn't need ports (it's a voltage source)
            grid_ports = {}
            dxdt_grid[i, :] = grid_core._dynamics_fn(grid_x, grid_ports, grid_meta)

        # Concatenate generator and grid derivatives
        return np.concatenate([dxdt.flatten(), dxdt_grid.flatten()])

    def _get_param(self, core, name, default):
        """Extract parameter from component subs dictionary"""
        for k, v in core.subs.items():
            if name in str(k):
                return v
        return default

    def initialize_equilibrium(self, run_power_flow=None):
        """
        Initialize dynamic simulation using power flow module.
        
        ALL initialization logic is in utils/power_flow.py for modularity.
        
        Args:
            run_power_flow (bool): If True, solve power flow first. 
                                  If None, uses auto_power_flow from config.
        
        Returns:
            x0: Initial state vector ready for simulation
        """
        # Determine if we should run power flow
        if run_power_flow is None:
            run_power_flow = self.auto_power_flow
        
        # Run power flow if requested and not already solved
        if run_power_flow and not self.power_flow_solved:
            print("\nRunning power flow for accurate initialization...")
            self.solve_power_flow(verbose=self.power_flow_verbose)
        
        # Use power flow module for ALL initialization
        from utils.power_flow import build_initial_state_vector
        
        x0 = build_initial_state_vector(
            self.builder,
            self.coordinator,
            verbose=True
        )
        
        # Verify initial conditions quality
        print("\n" + "="*70)
        print("  VERIFYING POWER FLOW BASED INITIAL CONDITIONS")
        print("="*70)
        
        dx0_check = self.dynamics(0.0, x0)
        max_deriv = np.max(np.abs(dx0_check))
        mean_deriv = np.mean(np.abs(dx0_check))
        
        print(f"\nInitial condition quality:")
        print(f"  Max |dx/dt|:  {max_deriv:.3e}")
        print(f"  Mean |dx/dt|: {mean_deriv:.3e}")
        
        if max_deriv < 1e-2:
            print(f"  [OK] Excellent initialization from power flow - ready for simulation!")
        elif max_deriv < 0.1:
            print(f"  [OK] Good initialization - minimal transients expected")
        else:
            print(f"  [!] Warning: Large initial derivatives - fast transients expected")
            print(f"      Consider checking power flow convergence or system parameters")
        
        print("="*70)
        
        return x0

    def simulate(self, x0, t_end=15.0, n_points=1500):
        """Run simulation"""
        fault_msg = f"fault @ Bus {self.fault_bus}, t={self.fault_start}-{self.fault_start + self.fault_duration}s" if self.fault_enabled else "no fault"
        print(f"\nSimulating {t_end}s, {fault_msg}")

        last_print = [0]
        def monitor(t, x):
            if t - last_print[0] > 0.5:
                # Extract generator states
                gen_size = self.n_gen * self.states_per_machine
                x_gen = x[:gen_size]
                x_r = x_gen.reshape(self.n_gen, self.states_per_machine)
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
        # Extract generator portion of state history
        gen_size = self.n_gen * self.states_per_machine
        x_gen_hist = sol.y[:gen_size, :].T  # Transpose to get time as first dimension
        x_hist = x_gen_hist.reshape(-1, self.n_gen, self.states_per_machine)
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
            
            # Extract Efd based on exciter type using output_fn
            exc_offset = self.n_gen_states
            exc_core = self.builder.exciters[g]
            
            if exc_core.output_fn is not None:
                # Use output function for each time point
                Efd = np.zeros(len(t))
                for i, ti in enumerate(t):
                    exc_x = x_hist[i, g, exc_offset:exc_offset+self.n_exc_states]
                    # Simple ports dict (not all ports needed for output)
                    exc_ports = {}
                    Efd[i] = exc_core.output_fn(exc_x, exc_ports, self.builder.exc_metadata[g])
            else:
                # Fallback: assume Efd is second state for most exciters
                Efd = x_hist[:, g, exc_offset + 1]
            
            # Governor states
            gov_offset = self.n_gen_states + self.n_exc_states
            x1 = x_hist[:, g, gov_offset + 0]
            x2 = x_hist[:, g, gov_offset + 1]

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
