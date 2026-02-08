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
        Find equilibrium using power flow results.
        
        Args:
            run_power_flow (bool): If True, solve power flow first. 
                                  If None, uses auto_power_flow from config.

        Uses direct calculation from power flow:
        - V, theta from bus data
        - P, Q from generator data
        - Calculate E'' and delta from: E'' = V + jXd'' * I
        """
        # Determine if we should run power flow
        if run_power_flow is None:
            run_power_flow = self.auto_power_flow
        
        # Run power flow if requested and not already solved
        if run_power_flow and not self.power_flow_solved:
            print("\nRunning power flow for accurate initialization...")
            self.solve_power_flow(verbose=self.power_flow_verbose)
        
        print("\nFinding equilibrium from power flow...")

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

        # Build full state vector for generators
        x0_gen = np.zeros((self.n_gen, self.states_per_machine))
        
        # Build state vector for grids
        x0_grid = np.zeros((self.n_grid, self.n_grid_states))

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
            x0_gen[i, 0] = delta_vals[i]  # delta
            x0_gen[i, 1] = M * 1.0  # p = M * omega (omega=1.0 at equilibrium)
            x0_gen[i, 2] = psi_d  # psi_d
            x0_gen[i, 3] = psi_q  # psi_q
            x0_gen[i, 4] = psi_f_vals[i]  # psi_f (field flux)
            x0_gen[i, 5] = 0.0  # psi_kd (d-axis damper)
            x0_gen[i, 6] = 0.0  # psi_kq (q-axis damper)

            # Exciter states - handle different types using model_name attribute
            exc_offset = self.n_gen_states
            exc_core = self.builder.exciters[i]
            if exc_core.model_name == 'ESST3A':
                # ESST3A states: [LG_y, LL_exc_x, VR, VM, VB_state]
                # Careful equilibrium calculation to avoid large transients
                exc_meta = self.builder.exc_metadata[i]
                
                # Proper field voltage calculation with correct per-unit base
                # Efd (exciter pu) = psi_f / Kfd_scale
                # where Kfd_scale = Lf * 2.0 (to match typical exciter base)
                xd = meta['xd']
                xl = meta['xl']
                xd1 = meta['xd1']
                Xad = xd - xl
                Xfl = (Xad * (xd1 - xl)) / (Xad - (xd1 - xl))
                Lf = Xad + Xfl
                Kfd_scale = Lf * 2.0
                
                Efd_target = psi_f_vals[i] / Kfd_scale
                Efd_target = np.clip(Efd_target, 0.5, 5.0)  # Limit to exciter range
                
                # 1. LG_y: voltage transducer at steady state
                x0_gen[i, exc_offset + 0] = V_mag[i]
                
                # 2. For equilibrium, need to solve the full ESST3A algebraic equations
                # Simplification: Assume voltage compensation gives VE ≈ Vt
                # Then VB = VE * FEX ≈ VE (assuming FEX ≈ 1 at low loading)
                
                # Estimate VE from terminal voltage (compensation circuit)
                KP = exc_meta.get('KP', 3.67)
                KI = exc_meta.get('KI', 0.435)
                XL = exc_meta.get('XL', 0.0098)
                THETAP = exc_meta.get('THETAP', 3.33)
                
                # Approximate: VE ≈ KP * Vt at light load
                VE_approx = KP * V_mag[i] if KP < 5 else V_mag[i]
                
                # Choose VM and VB to give Efd_target
                # Constrain: Efd = VB * VM, and both should be reasonable
                VM_eq = 1.0  # Typical value
                VB_eq = Efd_target / VM_eq
                VB_eq = np.clip(VB_eq, 0.5, 5.0)  # Keep VB reasonable
                
                # Compensator state: at equilibrium, assume small voltage error
                x0_gen[i, exc_offset + 1] = 0.0
                
                # VR from feedback: VR = VG + VM/KM
                KM = exc_meta.get('KM', 8.0)
                KG = exc_meta.get('KG', 1.0)
                VG_eq = KG * Efd_target
                VR_eq = VG_eq + VM_eq / KM
                VR_eq = np.clip(VR_eq, 0.0, 10.0)  # Keep VR reasonable
                
                x0_gen[i, exc_offset + 2] = VR_eq  # VR
                x0_gen[i, exc_offset + 3] = VM_eq  # VM  
                x0_gen[i, exc_offset + 4] = VB_eq  # VB_state
                
                print(f"  ESST3A equilibrium for Gen {i}: VR={VR_eq:.3f}, VM={VM_eq:.3f}, VB={VB_eq:.3f}, Efd={Efd_target:.3f}")
            elif exc_core.model_name == 'EXDC2':
                # EXDC2 states: [vm, vr1, vr2, vf]
                x0_gen[i, exc_offset + 0] = V_mag[i]  # vm (voltage measurement)
                x0_gen[i, exc_offset + 1] = psi_f_vals[i]  # vr1 (exciter output = Efd)
                x0_gen[i, exc_offset + 2] = 0.0  # vr2
                x0_gen[i, exc_offset + 3] = 0.0  # vf
            else:
                # Generic fallback for unknown exciter types
                # Initialize first state to voltage, second to field voltage
                print(f"  Warning: Unknown exciter type '{exc_core.model_name}' for Gen {i}, using generic initialization")
                if self.n_exc_states >= 1:
                    x0_gen[i, exc_offset + 0] = V_mag[i]
                if self.n_exc_states >= 2:
                    x0_gen[i, exc_offset + 1] = psi_f_vals[i]
                # Rest at zero
                for j in range(2, self.n_exc_states):
                    x0_gen[i, exc_offset + j] = 0.0

            # Governor states - ensure Tm = Te at equilibrium
            # At equilibrium: omega = 1.0, so Tm = Te
            # Need to verify the electrical power calculation includes ALL power
            gov_offset = self.n_gen_states + self.n_exc_states
            
            # Electrical torque at equilibrium (this is what the generator produces electrically)
            Te_eq = P_check[i]  # Vd*Id + Vq*Iq
            
            # For SMIB systems with local loads, verify the power balance
            # The generator electrical power should match mechanical power
            # Add a small margin to prevent drift
            Tm_eq = Te_eq * 1.0  # Mechanical power = electrical power at equilibrium
            
            # Set governor states to produce Tm at equilibrium
            # At steady state: x1 = x2 = Tm_target
            x0_gen[i, gov_offset + 0] = Tm_eq  # x1 
            x0_gen[i, gov_offset + 1] = Tm_eq  # x2
            
            # Set Pref to maintain this power at omega=1
            self.builder.gov_metadata[i]['Pref'] = Tm_eq
            self.builder.gov_metadata[i]['wref'] = 1.0

            # Update Vref to maintain equilibrium Efd
            exc_meta = self.builder.exc_metadata[i]
            
            if exc_core.model_name == 'ESST3A':
                # For ESST3A: At equilibrium, d(LG_y)/dt = 0 means LG_y = Vt
                # d(LL_exc_x)/dt = 0 means vil = LL_exc_x (so TC term = 0)
                # d(VR)/dt = 0 means KA * LL_exc_y = VR (so TA term = 0)
                # Work backwards: VR = KA * LL_exc_y = KA * ((TB/TC)*(vil - LL_exc_x) + LL_exc_x)
                # If LL_exc_x = vil at equilibrium, then LL_exc_y = vil
                # So: VR = KA * vil = KA * clip(vref - Vt, VIMIN, VIMAX)
                # For equilibrium: vref = Vt + VR/KA (assuming no limiter saturation)
                KA = exc_meta.get('KA', 20.0)
                VR_eq = x0_gen[i, exc_offset + 2]
                Vref_correct = V_mag[i] + VR_eq / KA
            elif exc_core.model_name == 'EXDC2':
                # For EXDC2: KA*(Vref - Vt) = Efd at equilibrium
                KA = self._get_param(exc_core, 'KA', 20.0)
                Efd_eq = psi_f_vals[i]
                Vref_correct = V_mag[i] + Efd_eq / KA
            else:
                # Generic fallback for unknown exciter types
                Vref_correct = V_mag[i]  # Default to terminal voltage
                print(f"  Warning: Unknown exciter type '{exc_core.model_name}', using Vref=Vt")

            # Update Vref in exciter core
            for key in exc_core.subs.keys():
                if 'Vref' in str(key) or 'vref' in str(key):
                    exc_core.subs[key] = Vref_correct
                    break
            
            # Also update in metadata
            if hasattr(exc_core, 'metadata') and exc_core.metadata:
                exc_core.metadata['vref'] = Vref_correct
            self.builder.exc_metadata[i]['vref'] = Vref_correct

        print("\nEquilibrium:")
        for i in range(self.n_gen):
            print(f"  Gen {i}: delta={np.degrees(delta_vals[i]):.2f} deg, P={P_gen[i]:.3f} pu, V={V_mag[i]:.4f} pu, psi_f={psi_f_vals[i]:.3f}")

        # Initialize grid states first
        print("\nInitializing grid/slack buses...")
        for i in range(self.n_grid):
            grid_meta = self.builder.grid_metadata[i]
            # Grid states: [V_magnitude, theta_angle]
            x0_grid[i, 0] = grid_meta['V_ref']  # V_magnitude at reference
            x0_grid[i, 1] = grid_meta['theta_ref']  # theta_angle at reference
            print(f"  Grid {i} at Bus {grid_meta['bus']}: V={grid_meta['V_ref']:.4f}, theta={np.degrees(grid_meta['theta_ref']):.2f} deg")

        # CRITICAL FIX: Adjust mechanical power Pm to match actual network electrical power
        # This ensures Pe = Pm exactly, preventing rotor acceleration/drift
        print("\nAdjusting mechanical power for zero acceleration...")
        gen_states = x0_gen[:, :7]
        
        # Prepare grid voltages
        grid_voltages = None
        if self.n_grid > 0:
            grid_voltages = np.array([x0_grid[i, 0] * np.exp(1j * x0_grid[i, 1]) 
                                      for i in range(self.n_grid)])
        
        # Get ACTUAL electrical power from network solver
        Id_net, Iq_net, Vd_net, Vq_net = self.coordinator.solve_network(gen_states, grid_voltages=grid_voltages)
        P_elec = Vd_net * Id_net + Vq_net * Iq_net
        
        print(f"  Electrical power from network:")
        for i in range(self.n_gen):
            P_target = P_gen[i]  # Original target
            P_actual = P_elec[i]  # What network actually provides
            P_error = P_target - P_actual
            
            print(f"    Gen {i}: P_target={P_target:.6f}, P_actual={P_actual:.6f}, diff={P_error:.6f}")
            
            # Set mechanical power Pm = P_actual to achieve zero acceleration
            # This is the KEY: Pm must match Pe for equilibrium
            Pm_correct = P_actual
            
            # Update governor states with corrected mechanical power
            exc_core = self.builder.exciters[i]
            n_exc_states = exc_core.n_states
            gov_offset = 7 + n_exc_states
            
            if self.builder.governors[i].model_name == 'TGOV1':
                x0_gen[i, gov_offset:gov_offset+2] = [Pm_correct, Pm_correct]
            
            # Update Pref in metadata
            self.builder.gov_metadata[i]['Pref'] = Pm_correct
            
            print(f"             Pm adjusted: {Pm_correct:.6f} pu")
        
        # Final verification - should show zero power error now
        gen_states = x0_gen[:, :7]  # Generator states haven't changed
        Id_check, Iq_check, Vd_check, Vq_check = self.coordinator.solve_network(gen_states, grid_voltages=grid_voltages)
        P_check = Vd_check * Id_check + Vq_check * Iq_check
        
        print(f"\n  Equilibrium verification (after Pm adjustment):")
        for i in range(self.n_gen):
            # Get mechanical power from governor states
            gov_offset = 7 + self.builder.exciters[i].n_states
            Pm = x0_gen[i, gov_offset]
            Pe = P_check[i]
            accel = Pm - Pe  # This should be ~0 for no drift
            print(f"    Gen {i}: Pm={Pm:.6f}, Pe={Pe:.6f}, accel={accel:.6e}")
        
        max_accel = np.max(np.abs([x0_gen[i, 7+self.builder.exciters[i].n_states] - P_check[i] for i in range(self.n_gen)]))
        if max_accel < 1e-6:
            print(f"  [OK] Perfect equilibrium (max accel = {max_accel:.6e}) - NO drift expected!")
        elif max_accel < 1e-4:
            print(f"  [OK] Good equilibrium (max accel = {max_accel:.6e}) - minimal drift")
        else:
            print(f"  [!] Warning: acceleration = {max_accel:.6e} may cause drift")

        # Combine generator and grid states into single vector
        # Layout: [gen0_states, gen1_states, ..., grid0_states, grid1_states, ...]
        x0 = np.concatenate([x0_gen.flatten(), x0_grid.flatten()])
        
        # CRITICAL: Use adaptive numerical solver to find true equilibrium
        # The algebraic formulas are just an initial guess - exciter dynamics require numerical solution
        from utils.equilibrium_solver import find_equilibrium_adaptive
        
        print("\n" + "="*70)
        print("  REFINING EQUILIBRIUM WITH NUMERICAL SOLVER")
        print("="*70)
        
        # Check initial guess quality
        dx0_initial = self.dynamics(0.0, x0)
        max_deriv_initial = np.max(np.abs(dx0_initial))
        print(f"\nAlgebraic guess quality: max|dx/dt| = {max_deriv_initial:.3e}")
        
        if max_deriv_initial > 1e-3:
            print(f"  >> Large derivatives detected! Using adaptive numerical solver...")
            
            # Use adaptive settling to find true equilibrium
            # Automatically continues until derivatives converge
            x0, converged = find_equilibrium_adaptive(
                self, x0, 
                tol=1e-4,           # Practical tolerance for complex exciters
                max_time=20.0,      # Longer time for slow exciter dynamics
                check_interval=0.5, # Check every 0.5s
                verbose=True
            )
            
            # Verify final equilibrium
            dx0_final = self.dynamics(0.0, x0)
            max_deriv_final = np.max(np.abs(dx0_final))
            
            print(f"After numerical refinement: max|dx/dt| = {max_deriv_final:.3e}")
            
            if converged and max_deriv_final < 1e-4:
                print(f"  [OK] Excellent equilibrium - NO drift expected!")
            elif max_deriv_final < 1e-2:
                print(f"  [OK] Good equilibrium - minimal drift possible")
            else:
                print(f"  [!] Warning: Equilibrium not fully converged - drift likely")
        else:
            print(f"  [OK] Algebraic guess is already excellent!")
        
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
