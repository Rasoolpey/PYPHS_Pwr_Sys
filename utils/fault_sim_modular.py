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
from utils.numba_kernels import prepare_jit_data, warmup_jit


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
        self.n_ren = len(self.builder.ren_generators)  # Number of WT3 converter units

        # Dynamically determine state counts from actual components using n_states attribute
        if len(self.builder.generators) > 0:
            self.n_gen_states = self.builder.generators[0].n_states
        else:
            self.n_gen_states = 7  # Fallback

        # Track individual component state counts (support heterogeneous components)
        self.gen_state_counts = [gen.n_states for gen in self.builder.generators]
        self.exc_state_counts = [exc.n_states for exc in self.builder.exciters]
        self.gov_state_counts = [gov.n_states for gov in self.builder.governors]
        self.grid_state_counts = [grid.n_states for grid in self.builder.grids] if self.n_grid > 0 else []

        # Build state index map for each machine
        self.machine_state_offsets = []  # Start index for each machine in the flat state vector
        offset = 0
        for i in range(self.n_gen):
            self.machine_state_offsets.append({
                'gen_start': offset,
                'exc_start': offset + self.gen_state_counts[i],
                'gov_start': offset + self.gen_state_counts[i] + self.exc_state_counts[i],
                'total': self.gen_state_counts[i] + self.exc_state_counts[i] + self.gov_state_counts[i]
            })
            offset += self.machine_state_offsets[i]['total']

        # Grid states come after all machine states
        self.grid_state_offset = offset
        offset += sum(self.grid_state_counts)

        # Renewable states come after grid states
        self.ren_state_offset = offset
        self._build_renewable_state_map(offset)

        # Base frequency
        self.omega_b = 2 * np.pi * 60

        # Load simulation configuration
        self._load_simulation_config(simulation_json)
        
        # Power flow solver (initialized but not run by default)
        self.power_flow_solved = False

        # Extract initial power flow data from JSON
        self._extract_initial_conditions()

        print(f"\nModular Fault Simulator: {self.n_gen} generators, {self.n_grid} grids, "
              f"{self.n_ren} renewables, {self.total_states} states")
        print(f"  Component states per machine: Gen={self.gen_state_counts}, Exc={self.exc_state_counts}, Gov={self.gov_state_counts}")
        if len(self.builder.exciters) > 0:
            exc_models = set(exc.model_name for exc in self.builder.exciters)
            print(f"  Exciter models: {', '.join(exc_models)}")
        if len(self.builder.governors) > 0:
            gov_models = set(gov.model_name for gov in self.builder.governors)
            print(f"  Governor models: {', '.join(gov_models)}")
    
    def _build_renewable_state_map(self, offset):
        """Build state offset map for renewable WT3 sub-components.

        Each WT3 unit has: REGCA1(3) + REECA1(4) + REPCA1(5) + WTDTA1(3) + WTTQA1(3) + WTPTA1(3) = 21 states
        WTARA1 has 0 dynamic states (algebraic).

        The mapping follows the builder's get_renewable_mapping() which links sub-components.
        """
        self.ren_mapping = self.builder.get_renewable_mapping() if self.n_ren > 0 else []
        self.ren_unit_offsets = []  # Per WT3 unit state offsets

        for r in range(self.n_ren):
            m = self.ren_mapping[r]
            unit = {'start': offset}

            # REGCA1 states
            unit['regca1_start'] = offset
            unit['regca1_n'] = self.builder.ren_generators[r].n_states  # 3
            offset += unit['regca1_n']

            # REECA1 states
            ree_idx = m.get('reeca1_idx')
            if ree_idx is not None:
                unit['reeca1_start'] = offset
                unit['reeca1_n'] = self.builder.ren_exciters[ree_idx].n_states  # 4
                offset += unit['reeca1_n']
            else:
                unit['reeca1_start'] = offset
                unit['reeca1_n'] = 0

            # REPCA1 states
            rep_idx = m.get('repca1_idx')
            if rep_idx is not None:
                unit['repca1_start'] = offset
                unit['repca1_n'] = self.builder.ren_plants[rep_idx].n_states  # 5
                offset += unit['repca1_n']
            else:
                unit['repca1_start'] = offset
                unit['repca1_n'] = 0

            # WTDTA1 states
            dt_idx = m.get('wtdta1_idx')
            if dt_idx is not None:
                unit['wtdta1_start'] = offset
                unit['wtdta1_n'] = self.builder.ren_drivetrains[dt_idx].n_states  # 3
                offset += unit['wtdta1_n']
            else:
                unit['wtdta1_start'] = offset
                unit['wtdta1_n'] = 0

            # WTTQA1 states
            tq_idx = m.get('wttqa1_idx')
            if tq_idx is not None:
                unit['wttqa1_start'] = offset
                unit['wttqa1_n'] = self.builder.ren_torque[tq_idx].n_states  # 3
                offset += unit['wttqa1_n']
            else:
                unit['wttqa1_start'] = offset
                unit['wttqa1_n'] = 0

            # WTPTA1 states
            pt_idx = m.get('wtpta1_idx')
            if pt_idx is not None:
                unit['wtpta1_start'] = offset
                unit['wtpta1_n'] = self.builder.ren_pitch[pt_idx].n_states  # 3
                offset += unit['wtpta1_n']
            else:
                unit['wtpta1_start'] = offset
                unit['wtpta1_n'] = 0

            unit['total'] = offset - unit['start']
            self.ren_unit_offsets.append(unit)

        self.total_states = offset

    def _load_simulation_config(self, simulation_json):
        """Load simulation configuration from JSON file or use defaults"""
        if simulation_json:
            print(f"Loading simulation config from: {simulation_json}")
            with open(simulation_json, 'r') as f:
                sim_config = json.load(f)
        else:
            print("Using default simulation configuration")
            sim_config = {}

        # Backward compatibility: support legacy keys used by older test cases
        # Legacy schema example:
        # {
        #   "simulation": {"t_end": 15.0, "num_points": 3000, "solver": "Radau", "rtol": 1e-6, "atol": 1e-8},
        #   "fault": {"enabled": true, "bus": 9, "t_start": 1.0, "t_end": 1.1, "impedance": [0.0, 0.0001]},
        #   "power_flow": {"enabled": true}
        # }
        if 'time' not in sim_config and 'simulation' in sim_config:
            legacy_sim = sim_config.get('simulation', {})
            sim_config['time'] = {
                't_end': legacy_sim.get('t_end', legacy_sim.get('tstop', 15.0)),
                'n_points': legacy_sim.get('num_points', legacy_sim.get('n_points', 3000)),
            }
            sim_config['solver'] = {
                'method': legacy_sim.get('solver', legacy_sim.get('method', 'Radau')),
                'rtol': legacy_sim.get('rtol', 1e-6),
                'atol': legacy_sim.get('atol', 1e-8),
            }

        if 'initialization' not in sim_config and 'power_flow' in sim_config:
            legacy_pf = sim_config.get('power_flow', {})
            sim_config['initialization'] = {
                'run_power_flow': legacy_pf.get('enabled', False),
                'power_flow_verbose': legacy_pf.get('verbose', True),
            }

        fault_cfg = sim_config.get('fault', {})
        if fault_cfg and ('start_time' not in fault_cfg) and ('t_start' in fault_cfg or 't_end' in fault_cfg or 'impedance' in fault_cfg):
            t_start = fault_cfg.get('t_start', 1.0)
            t_end = fault_cfg.get('t_end', t_start + 0.1)
            duration = max(0.0, float(t_end) - float(t_start))

            impedance = fault_cfg.get('impedance', None)
            if isinstance(impedance, (list, tuple)) and len(impedance) >= 2:
                impedance_real = float(impedance[0])
                impedance_imag = float(impedance[1])
            else:
                impedance_real = fault_cfg.get('impedance_real', 0.0)
                impedance_imag = fault_cfg.get('impedance_imag', 0.005)

            sim_config['fault'] = {
                **fault_cfg,
                'enabled': fault_cfg.get('enabled', True),
                'bus': fault_cfg.get('bus', 2),
                'start_time': t_start,
                'duration': duration,
                'impedance_real': impedance_real,
                'impedance_imag': impedance_imag,
            }
        
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
        """System dynamics - fully generic for heterogeneous component state counts"""
        # Extract states for each machine using individual offsets
        gen_states = []
        exc_states = []
        gov_states = []
        
        for i in range(self.n_gen):
            offsets = self.machine_state_offsets[i]
            gen_start = offsets['gen_start']
            exc_start = offsets['exc_start']
            gov_start = offsets['gov_start']
            machine_end = gen_start + offsets['total']
            
            gen_states.append(x_flat[gen_start:gen_start + self.gen_state_counts[i]])
            exc_states.append(x_flat[exc_start:exc_start + self.exc_state_counts[i]])
            gov_states.append(x_flat[gov_start:gov_start + self.gov_state_counts[i]])
        
        # Extract grid states (come after all machine states)
        x_grid = []
        if self.n_grid > 0:
            for i in range(self.n_grid):
                grid_start = self.grid_state_offset + sum(self.grid_state_counts[:i])
                grid_end = grid_start + self.grid_state_counts[i]
                x_grid.append(x_flat[grid_start:grid_end])

        # Determine if fault is active
        use_fault = self.fault_enabled and self.fault_start <= t < (self.fault_start + self.fault_duration)

        # Extract grid voltages if grids exist (generic for any grid state structure)
        grid_voltages = None
        if self.n_grid > 0:
            # Grid voltages are complex phasors: V = V_mag * exp(j*theta)
            grid_voltages = np.array([x_grid[i][0] * np.exp(1j * x_grid[i][1]) 
                                      for i in range(self.n_grid)])

        # Extract renewable states
        ren_states = {}  # keyed by sub-component type per unit
        for r in range(self.n_ren):
            u = self.ren_unit_offsets[r]
            m = self.ren_mapping[r]
            ren_states[r] = {
                'regca1': x_flat[u['regca1_start']:u['regca1_start'] + u['regca1_n']],
            }
            if u['reeca1_n'] > 0:
                ren_states[r]['reeca1'] = x_flat[u['reeca1_start']:u['reeca1_start'] + u['reeca1_n']]
            if u['repca1_n'] > 0:
                ren_states[r]['repca1'] = x_flat[u['repca1_start']:u['repca1_start'] + u['repca1_n']]
            if u['wtdta1_n'] > 0:
                ren_states[r]['wtdta1'] = x_flat[u['wtdta1_start']:u['wtdta1_start'] + u['wtdta1_n']]
            if u['wttqa1_n'] > 0:
                ren_states[r]['wttqa1'] = x_flat[u['wttqa1_start']:u['wttqa1_start'] + u['wttqa1_n']]
            if u['wtpta1_n'] > 0:
                ren_states[r]['wtpta1'] = x_flat[u['wtpta1_start']:u['wtpta1_start'] + u['wtpta1_n']]

        # Build renewable current injections for network solver
        ren_injections = None
        if self.n_ren > 0:
            ren_injections = []
            for r in range(self.n_ren):
                reg_x = ren_states[r]['regca1']
                reg_core = self.builder.ren_generators[r]
                reg_meta = self.builder.ren_gen_metadata[r]
                # Get current outputs from REGCA1
                # Use REGCA1's own filtered voltage state as the best available
                # estimate of terminal voltage before the network solve.
                V_est = float(abs(reg_x[2])) if len(reg_x) >= 3 else 1.0
                V_est = V_est if V_est > 0.01 else 1.0
                reg_out = reg_core.output_fn(reg_x, {'V': V_est}, reg_meta)
                ren_injections.append({
                    'Ip': reg_out['Ipout'],
                    'Iq': reg_out['Iqout'],
                })

        # Solve network with grid voltages as boundary conditions
        # Convert gen_states list to numpy array (coordinator expects 2D array)
        gen_states_array = np.array([gen_x[:min(7, len(gen_x))] for gen_x in gen_states])

        result = self.coordinator.solve_network(
            gen_states_array,
            grid_voltages=grid_voltages,
            ren_injections=ren_injections,
            use_fault=use_fault,
            fault_bus=self.fault_bus,
            fault_impedance=self.fault_impedance
        )
        Id, Iq, Vd, Vq, V_ren = result

        # ========================================================================
        # CENTER OF INERTIA (COI) FREQUENCY CALCULATION
        # ========================================================================
        # To prevent collective drift, we enforce that the COI frequency stays at
        # exactly 1.0 pu by removing any net acceleration.
        # 
        # The COI reference frame constraint: Σ(Mi * omegai) = M_total * omega_ref
        # where omega_ref = 1.0 (synchronous reference)
        #
        # This means: d/dt[Σ(Mi * omegai)] = 0
        # Which requires: Σ(dpi/dt) = 0
        omega_list = []
        M_list = []
        for i in range(self.n_gen):
            gen_meta_dict = self.builder.gen_metadata[i]
            M_i = gen_meta_dict['M']
            p_i = gen_states[i][1]
            omega_i = p_i / M_i
            omega_list.append(omega_i)
            M_list.append(M_i)
        
        M_total = sum(M_list)
        if M_total > 0:
            # COI frequency: weighted average of all generator frequencies
            omega_coi = sum(M_i * omega_i for M_i, omega_i in zip(M_list, omega_list)) / M_total
        else:
            # No inertia: use synchronous reference
            omega_coi = 1.0

        # Component dynamics - fully generic, works with any component state counts
        dxdt_per_machine = []  # List of derivative vectors, one per machine

        for i in range(self.n_gen):
            # Extract states for each component
            gen_x = gen_states[i]
            exc_x = exc_states[i]
            gov_x = gov_states[i]

            # Use JIT-compiled path if available
            if hasattr(self, 'use_jit') and self.use_jit and self.jit_data['gen_dyn_jit'][i] is not None:
                jd = self.jit_data
                gen_meta_arr = jd['gen_meta_arr'][i]
                exc_meta_arr = jd['exc_meta_arr'][i]
                gov_meta_arr = jd['gov_meta_arr'][i]

                # Auxiliary quantities
                omega = gen_x[1] / gen_meta_arr[0]  # p / M (M is index 0)
                Vt = np.sqrt(Vd[i]**2 + Vq[i]**2)

                # Exciter output (Efd) via JIT
                exc_out_fn = jd['exc_out_jit'][i]
                if exc_out_fn is not None:
                    ep = jd['exc_ports_buf'][i]
                    exc_model = self.builder.exciters[i].model_name
                    if exc_model == 'EXDC2':
                        ep[0] = Vt
                    elif exc_model == 'EXST1':
                        ep[0] = Vt; ep[1] = gen_x[4]
                    elif exc_model == 'ESST3A':
                        ep[0] = Vt; ep[1] = Id[i]; ep[2] = Iq[i]
                        ep[3] = Vd[i]; ep[4] = Vq[i]; ep[5] = gen_x[4]
                    elif exc_model == 'IEEEX1':
                        ep[0] = Vt; ep[1] = omega
                    Efd = exc_out_fn(exc_x, ep, exc_meta_arr)
                else:
                    Efd = exc_x[min(1, len(exc_x)-1)] if len(exc_x) > 1 else 1.0

                # Governor output (Tm) via JIT
                gov_out_fn = jd['gov_out_jit'][i]
                gp = jd['gov_ports_buf'][i]
                gov_model = self.builder.governors[i].model_name
                if gov_out_fn is not None:
                    if gov_model == 'TGOV1':
                        gp[0] = omega
                    elif gov_model == 'IEEEG1':
                        gp[0] = omega; gp[1] = 0.0
                    Tm = gov_out_fn(gov_x, gp, gov_meta_arr)
                else:
                    Tm = gov_x[-1] if len(gov_x) > 0 else 1.0

                # Governor dynamics via JIT
                gov_dxdt = jd['gov_dyn_jit'][i](gov_x, gp, gov_meta_arr)

                # Generator dynamics via JIT
                genbuf = jd['gen_ports_buf'][i]
                genbuf[0] = Id[i]; genbuf[1] = Iq[i]
                genbuf[2] = Vd[i]; genbuf[3] = Vq[i]
                genbuf[4] = Tm; genbuf[5] = Efd
                gen_dxdt = jd['gen_dyn_jit'][i](gen_x, genbuf, gen_meta_arr)
                
                # Apply COI correction to rotor angle and momentum derivatives
                # This prevents collective drift in multi-machine systems
                omega_b_jit = gen_meta_arr[3]  # omega_b is at index 3
                D_jit = gen_meta_arr[1]  # D is at index 1
                gen_dxdt[0] = omega_b_jit * (omega - omega_coi)
                # Correct damping term: D * (omega - omega_ref) where omega_ref = omega_coi
                Te = Vd[i] * Id[i] + Vq[i] * Iq[i]
                gen_dxdt[1] = Tm - Te - D_jit * (omega - omega_coi)

                # Exciter dynamics via JIT (reuse exc_ports_buf already filled)
                exc_dyn_fn = jd['exc_dyn_jit'][i]
                exc_dxdt = exc_dyn_fn(exc_x, ep, exc_meta_arr)

            else:
                # Original dict-based path (fallback)
                gen_core = self.builder.generators[i]
                exc_core = self.builder.exciters[i]
                gov_core = self.builder.governors[i]

                gen_meta = self.builder.gen_metadata[i]
                exc_meta = self.builder.exc_metadata[i]
                gov_meta = self.builder.gov_metadata[i]

                delta, p = gen_x[0], gen_x[1]
                M = gen_meta['M']
                omega = p / M
                Vt = np.sqrt(Vd[i]**2 + Vq[i]**2)

                exc_ports_for_output = {
                    'Vt': Vt, 'Vd': Vd[i], 'Vq': Vq[i],
                    'Id': Id[i], 'Iq': Iq[i],
                    'XadIfd': gen_x[4]
                }
                if exc_core.output_fn is not None:
                    Efd = exc_core.output_fn(exc_x, exc_ports_for_output, exc_meta)
                else:
                    Efd = exc_x[min(1, len(exc_x)-1)] if len(exc_x) > 1 else 1.0

                gov_ports = {'omega': omega}
                if gov_core.output_fn is not None:
                    Tm = gov_core.output_fn(gov_x, gov_ports, gov_meta)
                else:
                    Tm = gov_x[-1] if len(gov_x) > 0 else 1.0

                gov_dxdt = gov_core._dynamics_fn(gov_x, gov_ports, gov_meta)

                gen_ports = {
                    'Id': Id[i], 'Iq': Iq[i],
                    'Vd': Vd[i], 'Vq': Vq[i],
                    'Tm': Tm, 'Efd': Efd
                }
                gen_dxdt = gen_core._dynamics_fn(gen_x, gen_ports, gen_meta)
                
                # Apply COI correction to rotor angle and momentum derivatives
                # This prevents collective drift in multi-machine systems
                omega_b = gen_meta['omega_b']
                D = gen_meta['D']
                gen_dxdt[0] = omega_b * (omega - omega_coi)
                # Correct damping term: D * (omega - omega_ref) where omega_ref = omega_coi
                Te = Vd[i] * Id[i] + Vq[i] * Iq[i]
                gen_dxdt[1] = Tm - Te - D * (omega - omega_coi)

                exc_ports = {
                    'Vt': Vt, 'Vd': Vd[i], 'Vq': Vq[i],
                    'Id': Id[i], 'Iq': Iq[i],
                    'XadIfd': gen_x[4],
                    'Efd_fb': Efd
                }
                exc_dxdt = exc_core._dynamics_fn(exc_x, exc_ports, exc_meta)

            # Assemble derivative vector for this machine (generic - handles any state count)
            machine_dxdt = np.concatenate([gen_dxdt, exc_dxdt, gov_dxdt])
            dxdt_per_machine.append(machine_dxdt)

        # ========================================================================
        # ENFORCE COI CONSTRAINT: d(omega_COI)/dt = 0
        # ========================================================================
        # The COI frequency must remain at 1.0 to prevent collective drift.
        # We achieve this by ensuring Σ(dpi/dt) = 0.
        # 
        # Calculate net momentum acceleration
        if M_total > 0 and self.n_gen > 0:
            net_dp_dt = sum(dxdt_per_machine[i][1] for i in range(self.n_gen))  # Sum of all dp/dt
            net_accel_coi = net_dp_dt / M_total  # Net COI acceleration
            
            # If there's net acceleration, remove it by adjusting each generator's dp/dt
            # proportionally to their inertia (so larger machines contribute more to correction)
            if abs(net_accel_coi) > 1e-16:
                for i in range(self.n_gen):
                    M_i = M_list[i]
                    # Subtract the proportional share of net acceleration
                    correction = M_i * net_accel_coi
                    dxdt_per_machine[i][1] -= correction

        # Grid dynamics - maintain constant voltage (generic for any grid state count)
        dxdt_grid = []
        for i in range(self.n_grid):
            grid_core = self.builder.grids[i]
            grid_meta = self.builder.grid_metadata[i]
            grid_x_i = x_grid[i]
            
            # Grid doesn't need ports (it's a voltage source)
            grid_ports = {}
            grid_dxdt_i = grid_core._dynamics_fn(grid_x_i, grid_ports, grid_meta)
            dxdt_grid.append(grid_dxdt_i)

        # Renewable dynamics - WT3 sub-component interconnections
        dxdt_ren = []
        for r in range(self.n_ren):
            m = self.ren_mapping[r]
            u = self.ren_unit_offsets[r]
            V_conv = abs(V_ren[r]) if abs(V_ren[r]) > 0.01 else 1.0

            # --- Get outputs from each sub-component (bottom-up) ---

            # REGCA1 output: Pe, Qe, Ipout, Iqout
            reg_core = self.builder.ren_generators[r]
            reg_meta = self.builder.ren_gen_metadata[r]
            reg_x = ren_states[r]['regca1']
            reg_out = reg_core.output_fn(reg_x, {'V': V_conv}, reg_meta)
            Pe_conv = reg_out['Pe']
            Qe_conv = reg_out['Qe']

            # WTDTA1 output: wt, wg
            wt, wg = 1.0, 1.0
            dt_idx = m.get('wtdta1_idx')
            if dt_idx is not None and 'wtdta1' in ren_states[r]:
                dt_core = self.builder.ren_drivetrains[dt_idx]
                dt_meta = self.builder.ren_dt_metadata[dt_idx]
                dt_x = ren_states[r]['wtdta1']
                dt_out = dt_core.output_fn(dt_x, {}, dt_meta)
                wt, wg = dt_out['wt'], dt_out['wg']

            # WTPTA1 output: theta (pitch angle)
            theta = 0.0
            pt_idx = m.get('wtpta1_idx')
            if pt_idx is not None and 'wtpta1' in ren_states[r]:
                pt_core = self.builder.ren_pitch[pt_idx]
                pt_meta = self.builder.ren_pitch_metadata[pt_idx]
                pt_x = ren_states[r]['wtpta1']
                pt_out = pt_core.output_fn(pt_x, {}, pt_meta)
                theta = pt_out['theta']

            # WTARA1 output: Pm (aerodynamic power)
            Pm_aero = Pe_conv  # Default if no aero model
            aero_idx = m.get('wtara1_idx')
            if aero_idx is not None:
                aero_core = self.builder.ren_aero[aero_idx]
                aero_meta = self.builder.ren_aero_metadata[aero_idx]
                aero_out = aero_core.output_fn(None, {'theta': theta, 'wt': wt}, aero_meta)
                Pm_aero = aero_out['Paero']  # Extract Paero from output dict

            # WTTQA1 output: Pref, wref
            Pref, wref = Pe_conv, 1.0
            tq_idx = m.get('wttqa1_idx')
            if tq_idx is not None and 'wttqa1' in ren_states[r]:
                tq_core = self.builder.ren_torque[tq_idx]
                tq_meta = self.builder.ren_torque_metadata[tq_idx]
                tq_x = ren_states[r]['wttqa1']
                tq_out = tq_core.output_fn(tq_x, {'Pe': Pe_conv, 'wg': wg}, tq_meta)
                Pref = tq_out['Pref']
                wref = tq_out['wref']

            # REPCA1 output: Pext, Qext
            Pext, Qext = 0.0, 0.0
            rep_idx = m.get('repca1_idx')
            if rep_idx is not None and 'repca1' in ren_states[r]:
                rep_core = self.builder.ren_plants[rep_idx]
                rep_meta = self.builder.ren_plant_metadata[rep_idx]
                rep_x = ren_states[r]['repca1']
                # CRITICAL: REPCA1 needs measured powers (Pline, Qline) for feedback control
                rep_out = rep_core.output_fn(rep_x, {
                    'V': V_conv, 'f': wg, 'Pline': Pe_conv, 'Qline': Qe_conv
                }, rep_meta)
                Pext = rep_out['Pext']
                Qext = rep_out['Qext']

            # REECA1 output: Ipcmd, Iqcmd
            Ipcmd, Iqcmd = reg_x[0], reg_x[1]  # Default: current state
            ree_idx = m.get('reeca1_idx')
            if ree_idx is not None and 'reeca1' in ren_states[r]:
                ree_core = self.builder.ren_exciters[ree_idx]
                ree_meta = self.builder.ren_exc_metadata[ree_idx]
                ree_x = ren_states[r]['reeca1']
                ree_out = ree_core.output_fn(ree_x, {
                    'V': V_conv, 'Pe': Pe_conv, 'Qe': Qe_conv, 'wg': wg
                }, ree_meta)
                Ipcmd = ree_out['Ipcmd']
                Iqcmd = ree_out['Iqcmd']

            # --- Compute dynamics for each sub-component (using interconnected ports) ---
            unit_dxdt = []

            # REGCA1 dynamics
            reg_dxdt = reg_core._dynamics_fn(reg_x, {
                'Ipcmd': Ipcmd, 'Iqcmd': Iqcmd, 'V': V_conv
            }, reg_meta)
            unit_dxdt.append(reg_dxdt)

            # REECA1 dynamics
            if ree_idx is not None and 'reeca1' in ren_states[r]:
                ree_dxdt = ree_core._dynamics_fn(ree_x, {
                    'V': V_conv, 'Pe': Pe_conv, 'Qe': Qe_conv,
                    'Pext': Pext, 'Qext': Qext, 'wg': wg
                }, ree_meta)
                unit_dxdt.append(ree_dxdt)

            # REPCA1 dynamics
            if rep_idx is not None and 'repca1' in ren_states[r]:
                rep_dxdt = rep_core._dynamics_fn(rep_x, {
                    'V': V_conv, 'f': wg, 'Pline': Pe_conv, 'Qline': Qe_conv
                }, rep_meta)
                unit_dxdt.append(rep_dxdt)

            # WTDTA1 dynamics
            if dt_idx is not None and 'wtdta1' in ren_states[r]:
                dt_dxdt = dt_core._dynamics_fn(dt_x, {
                    'Pm': Pm_aero, 'Pe': Pe_conv
                }, dt_meta)
                unit_dxdt.append(dt_dxdt)

            # WTTQA1 dynamics
            if tq_idx is not None and 'wttqa1' in ren_states[r]:
                tq_dxdt = tq_core._dynamics_fn(tq_x, {
                    'Pe': Pe_conv, 'wg': wg
                }, tq_meta)
                unit_dxdt.append(tq_dxdt)

            # WTPTA1 dynamics
            if pt_idx is not None and 'wtpta1' in ren_states[r]:
                Pord = ren_states[r].get('reeca1', np.array([0, 0, 0, Pe_conv]))[3] if 'reeca1' in ren_states[r] else Pe_conv
                pt_dxdt = pt_core._dynamics_fn(pt_x, {
                    'wt': wt, 'Pord': Pord, 'Pref': Pref
                }, pt_meta)
                unit_dxdt.append(pt_dxdt)

            dxdt_ren.append(np.concatenate(unit_dxdt))

        # Flatten all derivatives into a single vector (fully generic)
        dxdt_flat = np.concatenate(dxdt_per_machine + dxdt_grid + dxdt_ren)
        return dxdt_flat

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

        # Post-initialization equilibrium correction
        print("\nRefining equilibrium with full dynamics check...")
        
        for iter_refine in range(10):
            # 1. Calculate derivatives
            dxdt = self.dynamics(0.0, x0)
            
            # 2. Prepare network data for init_fn calls
            # (Re-solve network to get V, I consistent with current states)
            gen_states = []
            for i in range(self.n_gen):
                gs = self.machine_state_offsets[i]['gen_start']
                gen_states.append(x0[gs:gs + self.gen_state_counts[i]])
            gen_states_array = np.array([gx[:min(7, len(gx))] for gx in gen_states])
            
            x_grid = []
            if self.n_grid > 0:
                for i in range(self.n_grid):
                    grid_start = self.grid_state_offset + sum(self.grid_state_counts[:i])
                    x_grid.append(x0[grid_start:grid_start + self.grid_state_counts[i]])
            
            grid_voltages = None
            if self.n_grid > 0:
                grid_voltages = np.array([x_grid[i][0] * np.exp(1j * x_grid[i][1]) for i in range(self.n_grid)])
                
            ren_injections = None
            if self.n_ren > 0:
                ren_injections = []
                for r in range(self.n_ren):
                    u = self.ren_unit_offsets[r]
                    reg_x = x0[u['regca1_start']:u['regca1_start'] + u['regca1_n']]
                    reg_core = self.builder.ren_generators[r]
                    reg_meta = self.builder.ren_gen_metadata[r]
                    V_est = float(abs(reg_x[2])) if len(reg_x) >= 3 else 1.0
                    V_est = V_est if V_est > 0.01 else 1.0
                    reg_out = reg_core.output_fn(reg_x, {'V': V_est}, reg_meta)
                    ren_injections.append({'Ip': reg_out['Ipout'], 'Iq': reg_out['Iqout']})

            Id, Iq, Vd, Vq, V_ren = self.coordinator.solve_network(
                gen_states_array, grid_voltages=grid_voltages, ren_injections=ren_injections
            )
            
            max_dpsi = 0.0
            max_dp = 0.0
            updates_made = False
            
            # 3. Correct Exciters (dpsi_f/dt -> Efd)
            for i in range(self.n_gen):
                gs = self.machine_state_offsets[i]['gen_start']
                psi_f = x0[gs + 4]
                gen_meta = self.builder.gen_metadata[i]
                
                # Calculate Kfd scaling factor
                xd = gen_meta['xd']; xl = gen_meta['xl']; xd1 = gen_meta['xd1']
                Xad = xd - xl
                Xfl = (Xad * (xd1 - xl)) / (Xad - (xd1 - xl))
                Kfd_scale = (Xad + Xfl) / max(Xad, 1e-6)
                
                # Target Efd to maintain current psi_f
                Efd_target = psi_f / Kfd_scale
                
                exc_start = self.machine_state_offsets[i]['exc_start']
                exc_core = self.builder.exciters[i]
                exc_meta = self.builder.exc_metadata[i]
                Vt = np.sqrt(Vd[i]**2 + Vq[i]**2)
                
                # Iteratively solve for exciter states that produce exactly Efd_target
                # This handles non-linearities (like FEX in ESST3A) that cause init_fn mismatch
                if exc_core.init_fn is not None:
                    Efd_req = Efd_target
                    best_exc_x = None
                    best_err = 1e9
                    
                    for _ in range(15):  # Newton-like iteration
                        exc_x_new = exc_core.init_fn(
                            Efd_eq=Efd_req, V_mag=Vt, 
                            Vd=Vd[i], Vq=Vq[i], Id=Id[i], Iq=Iq[i], psi_f=psi_f
                        )
                        
                        # Check output
                        exc_ports = {'Vt': Vt, 'Vd': Vd[i], 'Vq': Vq[i], 'Id': Id[i], 'Iq': Iq[i], 'XadIfd': psi_f}
                        if exc_core.output_fn is not None:
                            Efd_out = exc_core.output_fn(exc_x_new, exc_ports, exc_meta)
                        else:
                            Efd_out = exc_x_new[min(1, len(exc_x_new)-1)]
                        
                        err = Efd_out - Efd_target
                        if abs(err) < best_err:
                            best_err = abs(err)
                            best_exc_x = exc_x_new
                        
                        if abs(err) < 1e-7:
                            break
                        
                        # Feedback correction: if output is low, request more
                        Efd_req = Efd_req - err

                    # Update states with best found
                    if best_exc_x is not None:
                        x0[exc_start:exc_start + self.exc_state_counts[i]] = best_exc_x
                        updates_made = True

                    # Check final error
                    Efd_err = Efd_out - Efd_target
                    flux_err = abs(Efd_err * Kfd_scale)
                    
                    if flux_err > 1e-5:
                        max_dpsi = max(max_dpsi, flux_err)
                        
                        # Check if limits are constraining the output and relax them if needed
                        # This handles cases where JSON limits are too tight for the power flow solution
                        if Efd_target > Efd_out and exc_core.model_name in ['ESST3A', 'EXST1', 'IEEEX1', 'EXDC2']:
                            relaxed = False
                            
                            def relax_limit(key, target_val):
                                if key in exc_meta:
                                    # Relax conservatively: only 20% above target
                                    # Avoids destroying transient stability limits
                                    if exc_meta[key] < target_val * 1.2:
                                        old_lim = exc_meta[key]
                                        new_lim = target_val * 1.2
                                        exc_meta[key] = new_lim
                                        print(f"    Gen {i}: Relaxing {key} {old_lim:.3f} -> {new_lim:.3f}")
                                        return True
                                return False

                            # Relax limits that might be constraining Efd
                            relaxed |= relax_limit('VRMAX', Efd_target)
                            relaxed |= relax_limit('VMMAX', Efd_target)
                            relaxed |= relax_limit('Efd_max', Efd_target)

                            if exc_core.model_name == 'ESST3A':
                                relaxed |= relax_limit('VGMAX', Efd_target)
                                relaxed |= relax_limit('VBMAX', Efd_target)
                                # Do NOT relax VIMAX - it limits exciter input during
                                # transients and is critical for transient stability.
                                # If VIMAX is too low, increase it in the JSON config.
                            
                            if relaxed:
                                updates_made = True
                                # Sync metadata if core has its own copy
                                if hasattr(exc_core, 'metadata'):
                                    exc_core.metadata.update(exc_meta)
                                
                                # Re-run init with new limits immediately
                                if exc_core.init_fn is not None:
                                    exc_x_new = exc_core.init_fn(
                                        Efd_eq=Efd_target, V_mag=Vt, 
                                        Vd=Vd[i], Vq=Vq[i], Id=Id[i], Iq=Iq[i], psi_f=psi_f
                                    )
                                    x0[exc_start:exc_start + self.exc_state_counts[i]] = exc_x_new

                        if iter_refine == 0 and not updates_made:
                            print(f"    Gen {i}: Exciter mismatch Efd {Efd_target:.4f} vs {Efd_out:.4f} (err={Efd_err:.1e})")
            
            # 4. Correct Governors (dp/dt -> Pref)
            for i in range(self.n_gen):
                gs = self.machine_state_offsets[i]['gen_start']
                dp_dt = dxdt[gs + 1]
                if abs(dp_dt) > 1e-6:
                    max_dp = max(max_dp, abs(dp_dt))
                    gov_meta = self.builder.gov_metadata[i]
                    gov_core = self.builder.governors[i]
                    Pref_old = gov_meta.get('Pref', 1.0)
                    Pref_new = Pref_old - dp_dt
                    
                    gov_meta['Pref'] = Pref_new
                    if hasattr(gov_core, 'meta'):
                        gov_core.meta['Pref'] = Pref_new
                    
                    gov_start = self.machine_state_offsets[i]['gov_start']
                    if gov_core.init_fn is not None:
                        gov_x = gov_core.init_fn(Pm_eq=Pref_new)
                        for j in range(self.gov_state_counts[i]):
                            x0[gov_start + j] = gov_x[j]
                    updates_made = True

            if not updates_made:
                print(f"  Converged in {iter_refine} iterations.")
                break
            
            print(f"  Iter {iter_refine}: Max dpsi_f={max_dpsi:.3e}, Max dp={max_dp:.3e}")

        # Verify initial conditions quality
        print("\n" + "="*70)
        print("  VERIFYING POWER FLOW BASED INITIAL CONDITIONS")
        print("="*70)
        
        dx0_check = self.dynamics(0.0, x0)
        max_deriv = np.max(np.abs(dx0_check))
        mean_deriv = np.mean(np.abs(dx0_check))
        max_idx = np.argmax(np.abs(dx0_check))
        
        print(f"\nInitial condition quality:")
        print(f"  Max |dx/dt|:  {max_deriv:.3e} (at state index {max_idx}, value={dx0_check[max_idx]:.3e})")
        print(f"  Mean |dx/dt|: {mean_deriv:.3e}")
        
        # Identify which component has the largest derivative
        offset = 0
        for i in range(self.n_gen):
            offsets = self.machine_state_offsets[i]
            machine_start = offsets['gen_start']
            machine_end = machine_start + offsets['total']
            if machine_start <= max_idx < machine_end:
                local_idx = max_idx - machine_start
                if local_idx < self.gen_state_counts[i]:
                    comp_name = f"Gen {i} (state {local_idx})"
                elif local_idx < self.gen_state_counts[i] + self.exc_state_counts[i]:
                    exc_local = local_idx - self.gen_state_counts[i]
                    exc_model = self.builder.exciters[i].model_name
                    comp_name = f"Gen {i} {exc_model} exciter (state {exc_local})"
                else:
                    gov_local = local_idx - self.gen_state_counts[i] - self.exc_state_counts[i]
                    gov_model = self.builder.governors[i].model_name
                    comp_name = f"Gen {i} {gov_model} governor (state {gov_local})"
                print(f"  Largest derivative is in: {comp_name}")
        
        if max_deriv < 1e-2:
            print(f"  [OK] Excellent initialization from power flow - ready for simulation!")
        elif max_deriv < 0.1:
            print(f"  [OK] Good initialization - minimal transients expected")
        else:
            print(f"  [!] Warning: Large initial derivatives - fast transients expected")
            print(f"      Consider checking power flow convergence or system parameters")
        
        print("="*70)
        
        return x0

    def simulate(self, x0, t_end=15.0, n_points=1500, use_jit=True):
        """Run simulation"""
        fault_msg = f"fault @ Bus {self.fault_bus}, t={self.fault_start}-{self.fault_start + self.fault_duration}s" if self.fault_enabled else "no fault"
        print(f"\nSimulating {t_end}s, {fault_msg}")

        # Prepare JIT-compiled dynamics (pack metadata into arrays, warmup)
        if use_jit and self.n_gen > 0:
            try:
                print("  Preparing JIT-compiled dynamics...")
                self.jit_data = prepare_jit_data(self.builder)
                self.use_jit = True
                warmup_jit(self.jit_data, self.n_gen)
            except Exception as e:
                print(f"  JIT preparation failed ({e}), falling back to Python")
                self.use_jit = False
        else:
            self.use_jit = False

        last_print = [0]
        def monitor(t, x):
            if t - last_print[0] > 0.5:
                # Extract generator states generically using individual offsets
                omegas = []
                deltas = []
                Ms = []
                for i in range(self.n_gen):
                    offsets = self.machine_state_offsets[i]
                    gen_start = offsets['gen_start']
                    delta = x[gen_start]  # First state of generator is always delta
                    p = x[gen_start + 1]  # Second state is always momentum p
                    M = self.builder.gen_metadata[i]['M']
                    omega = p / M
                    omegas.append(omega)
                    deltas.append(delta)
                    Ms.append(M)
                
                omega_avg = np.mean(omegas)
                omega_dev = float(np.max(np.abs(np.array(omegas) - 1.0))) if len(omegas) else 0.0

                deltas_arr = np.array(deltas, dtype=float)
                deltas_unwrapped = np.unwrap(deltas_arr)
                weights = np.array(Ms, dtype=float)
                wsum = float(np.sum(weights)) if len(weights) else 0.0
                delta_coi = float(np.sum(weights * deltas_unwrapped) / wsum) if wsum > 0 else float(np.mean(deltas_unwrapped))
                deltas_deg = np.degrees(deltas_unwrapped)
                delta_min = float(np.min(deltas_deg)) if len(deltas_deg) else 0.0
                delta_max = float(np.max(deltas_deg)) if len(deltas_deg) else 0.0
                delta_rel_max = float(np.max(np.abs(np.degrees(deltas_unwrapped - delta_coi)))) if len(deltas_unwrapped) else 0.0

                if self.fault_enabled:
                    status = "FAULT" if self.fault_start <= t < (self.fault_start + self.fault_duration) else "normal"
                else:
                    status = "normal"

                print(
                    f"  t={t:.2f}s [{status:6s}]: omega={omega_avg:.6f} (dev={omega_dev:.6f}), "
                    f"delta_rel_max={delta_rel_max:.1f} deg, delta=[{delta_min:.1f}, {delta_max:.1f}] deg"
                )
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
        """Plot results - fully generic for heterogeneous component state counts"""
        t = sol.t
        x_hist = sol.y.T  # Shape: (n_time, n_states)
        
        # Extract time histories for each generator component (generic)
        delta_hist = np.zeros((len(t), self.n_gen))
        omega_hist = np.zeros((len(t), self.n_gen))
        Efd_hist = np.zeros((len(t), self.n_gen))
        Tm_hist = np.zeros((len(t), self.n_gen))
        
        for g in range(self.n_gen):
            offsets = self.machine_state_offsets[g]
            gen_start = offsets['gen_start']
            exc_start = offsets['exc_start']
            gov_start = offsets['gov_start']
            
            gen_meta = self.builder.gen_metadata[g]
            exc_meta = self.builder.exc_metadata[g]
            gov_meta = self.builder.gov_metadata[g]
            
            gen_core = self.builder.generators[g]
            exc_core = self.builder.exciters[g]
            gov_core = self.builder.governors[g]
            
            # Extract histories generically using actual state counts
            for i in range(len(t)):
                # Generator states: delta, p, ...
                gen_x = x_hist[i, gen_start:gen_start + self.gen_state_counts[g]]
                delta_hist[i, g] = gen_x[0]
                omega_hist[i, g] = gen_x[1] / gen_meta['M']
                
                # Exciter Efd using output function (generic)
                exc_x = x_hist[i, exc_start:exc_start + self.exc_state_counts[g]]
                if exc_core.output_fn is not None:
                    # Minimal ports for output function
                    Vt = 1.0  # Placeholder - exact value not critical for plotting
                    exc_ports = {'Vt': Vt, 'Vd': 0.0, 'Vq': Vt, 'Id': 0.0, 'Iq': 0.0, 'XadIfd': gen_x[4] if len(gen_x) > 4 else 1.0}
                    Efd_hist[i, g] = exc_core.output_fn(exc_x, exc_ports, exc_meta)
                else:
                    # Fallback: second state
                    Efd_hist[i, g] = exc_x[min(1, len(exc_x)-1)] if len(exc_x) > 1 else 1.0
                
                # Governor Tm using output function (generic - NO hardcoded equations)
                gov_x = x_hist[i, gov_start:gov_start + self.gov_state_counts[g]]
                if gov_core.output_fn is not None:
                    gov_ports = {'omega': omega_hist[i, g]}
                    Tm_hist[i, g] = gov_core.output_fn(gov_x, gov_ports, gov_meta)
                else:
                    # Fallback: last state
                    Tm_hist[i, g] = gov_x[-1] if len(gov_x) > 0 else 1.0

        fig, axes = plt.subplots(2, 3, figsize=(15, 8))
        colors = ['b', 'r', 'g', 'm', 'c', 'y', 'k']  # Support more generators

        if self.fault_enabled:
            for ax_row in axes:
                for ax in ax_row:
                    ax.axvspan(self.fault_start, self.fault_start + self.fault_duration, alpha=0.2, color='red')

        # Plot all generators using pre-extracted generic histories
        for g in range(self.n_gen):
            color = colors[g % len(colors)]
            axes[0, 0].plot(t, np.degrees(delta_hist[:, g]), color, label=f'Gen {g+1}', linewidth=1.5)
            axes[0, 1].plot(t, omega_hist[:, g], color, linewidth=1.5)
            axes[0, 2].plot(t, Efd_hist[:, g], color, linewidth=1.5)
            axes[1, 0].plot(t, Tm_hist[:, g], color, linewidth=1.5)
            axes[1, 1].plot(t, (omega_hist[:, g] - 1.0) * 60, color, linewidth=1.5)
            axes[1, 2].plot(t, np.degrees(delta_hist[:, g] - delta_hist[0, g]), color, linewidth=1.5)

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
        plt.close(fig)

        # --- Wind Turbine (WT3) dynamics plots ---
        if self.n_ren > 0:
            self._plot_renewable_results(sol, filename, output_dir)

    def _plot_renewable_results(self, sol, filename, output_dir):
        """Plot wind turbine (WT3) sub-component dynamics."""
        t = sol.t
        x_hist = sol.y.T  # (n_time, n_states)

        for r in range(self.n_ren):
            u = self.ren_unit_offsets[r]
            m = self.ren_mapping[r]

            # --- Extract state time histories ---
            # REGCA1 states: [Ip, Iq, Vfilter]
            reg_start = u['regca1_start']
            Ip_hist = x_hist[:, reg_start + 0]
            Iq_hist = x_hist[:, reg_start + 1]
            Vf_reg_hist = x_hist[:, reg_start + 2]

            # Compute Pe, Qe from REGCA1 output at each time step
            reg_core = self.builder.ren_generators[r]
            reg_meta = self.builder.ren_gen_metadata[r]
            Pe_hist = np.zeros(len(t))
            Qe_hist = np.zeros(len(t))
            for i in range(len(t)):
                reg_x_i = x_hist[i, reg_start:reg_start + u['regca1_n']]
                reg_out_i = reg_core.output_fn(reg_x_i, {'V': Vf_reg_hist[i]}, reg_meta)
                Pe_hist[i] = reg_out_i['Pe']
                Qe_hist[i] = reg_out_i['Qe']

            # REECA1 states: [Vf, Pf, piq_xi, Pord]
            Pord_hist = None
            if u['reeca1_n'] > 0:
                ree_start = u['reeca1_start']
                Vf_ree_hist = x_hist[:, ree_start + 0]
                Pf_ree_hist = x_hist[:, ree_start + 1]
                Pord_hist = x_hist[:, ree_start + 3]

            # REPCA1 states: [Vf, Qf, s2_xi, Pf, s5_xi]
            s5_xi_hist = None
            if u['repca1_n'] > 0:
                rep_start = u['repca1_start']
                Vf_rep_hist = x_hist[:, rep_start + 0]
                Qf_rep_hist = x_hist[:, rep_start + 1]
                s2_xi_hist = x_hist[:, rep_start + 2]
                Pf_rep_hist = x_hist[:, rep_start + 3]
                s5_xi_hist = x_hist[:, rep_start + 4]

            # WTDTA1 states: [theta_tw, p_t, p_g]
            wt_hist = None
            wg_hist = None
            theta_tw_hist = None
            if u['wtdta1_n'] > 0:
                dt_start = u['wtdta1_start']
                dt_idx = m.get('wtdta1_idx')
                dt_meta = self.builder.ren_dt_metadata[dt_idx]
                theta_tw_hist = x_hist[:, dt_start + 0]
                wt_hist = x_hist[:, dt_start + 1] / dt_meta['Jt']
                wg_hist = x_hist[:, dt_start + 2] / dt_meta['Jg']

            # WTTQA1 states: [Pef, wref, pi_xi]
            Pef_hist = None
            if u['wttqa1_n'] > 0:
                tq_start = u['wttqa1_start']
                Pef_hist = x_hist[:, tq_start + 0]

            # WTPTA1 states: [piw_xi, pic_xi, theta]
            pitch_hist = None
            if u['wtpta1_n'] > 0:
                pt_start = u['wtpta1_start']
                pitch_hist = x_hist[:, pt_start + 2]

            # --- Create figure: 3x3 ---
            fig, axes = plt.subplots(3, 3, figsize=(16, 10))
            fig.suptitle(f'Wind Turbine {r+1} (WT3) Dynamics - Bus {m.get("bus", "?")}',
                         fontsize=14, fontweight='bold')

            if self.fault_enabled:
                for ax_row in axes:
                    for ax in ax_row:
                        ax.axvspan(self.fault_start, self.fault_start + self.fault_duration,
                                   alpha=0.2, color='red')

            # Row 0: Converter electrical
            ax = axes[0, 0]
            ax.plot(t, Pe_hist, 'b-', label='Pe', linewidth=1.5)
            ax.plot(t, Qe_hist, 'r-', label='Qe', linewidth=1.5)
            ax.set_ylabel('Power (pu)')
            ax.set_title('REGCA1: Converter Power')
            ax.legend()
            ax.grid(True)

            ax = axes[0, 1]
            ax.plot(t, Ip_hist, 'b-', label='Ip', linewidth=1.5)
            ax.plot(t, Iq_hist, 'r-', label='Iq', linewidth=1.5)
            if Pord_hist is not None:
                # Ipcmd = Pord / V
                Ipcmd_approx = Pord_hist / np.maximum(Vf_reg_hist, 0.01)
                ax.plot(t, Ipcmd_approx, 'b--', label='Ipcmd', linewidth=1.0, alpha=0.7)
            ax.set_ylabel('Current (pu)')
            ax.set_title('REGCA1: Converter Currents')
            ax.legend()
            ax.grid(True)

            ax = axes[0, 2]
            ax.plot(t, Vf_reg_hist, 'b-', linewidth=1.5)
            ax.set_ylabel('Voltage (pu)')
            ax.set_title('REGCA1: Terminal Voltage')
            ax.grid(True)

            # Row 1: Control signals
            ax = axes[1, 0]
            if Pord_hist is not None:
                ax.plot(t, Pord_hist, 'b-', label='Pord (REECA1)', linewidth=1.5)
            if s5_xi_hist is not None:
                ax.plot(t, s5_xi_hist, 'r--', label='Pext=s5_xi (REPCA1)', linewidth=1.5)
            ax.set_ylabel('Power (pu)')
            ax.set_title('Control: Power Commands')
            ax.legend(fontsize=8)
            ax.grid(True)

            ax = axes[1, 1]
            if u['repca1_n'] > 0:
                ax.plot(t, Vf_rep_hist, 'b-', label='Vf (filtered)', linewidth=1.5)
                rep_meta = self.builder.ren_plant_metadata[m.get('repca1_idx')]
                ax.axhline(y=rep_meta.get('Vref0', 1.0), color='k', linestyle='--',
                           label=f'Vref0={rep_meta.get("Vref0", 1.0):.3f}', alpha=0.7)
            ax.set_ylabel('Voltage (pu)')
            ax.set_title('REPCA1: Voltage Control')
            ax.legend(fontsize=8)
            ax.grid(True)

            ax = axes[1, 2]
            if u['repca1_n'] > 0:
                ax.plot(t, Pf_rep_hist, 'b-', label='Pf (P filter)', linewidth=1.5)
                ax.plot(t, Qf_rep_hist, 'r-', label='Qf (Q filter)', linewidth=1.5)
                rep_meta = self.builder.ren_plant_metadata[m.get('repca1_idx')]
                ax.axhline(y=rep_meta.get('Pline0', 0.35), color='b', linestyle='--',
                           label=f'Pline0={rep_meta.get("Pline0", 0.35):.3f}', alpha=0.7)
            ax.set_ylabel('Power (pu)')
            ax.set_title('REPCA1: Power Filters')
            ax.legend(fontsize=8)
            ax.grid(True)

            # Row 2: Mechanical
            ax = axes[2, 0]
            if wt_hist is not None and wg_hist is not None:
                ax.plot(t, wt_hist, 'b-', label='wt (turbine)', linewidth=1.5)
                ax.plot(t, wg_hist, 'r-', label='wg (generator)', linewidth=1.5)
            ax.set_ylabel('Speed (pu)')
            ax.set_xlabel('Time (s)')
            ax.set_title('WTDTA1: Drive Train Speeds')
            ax.legend()
            ax.grid(True)

            ax = axes[2, 1]
            if theta_tw_hist is not None:
                ax.plot(t, np.degrees(theta_tw_hist), 'b-', linewidth=1.5)
            ax.set_ylabel('Twist (deg)')
            ax.set_xlabel('Time (s)')
            ax.set_title('WTDTA1: Shaft Twist')
            ax.grid(True)

            ax = axes[2, 2]
            if pitch_hist is not None:
                ax.plot(t, np.degrees(pitch_hist), 'b-', linewidth=1.5)
            ax.set_ylabel('Pitch (deg)')
            ax.set_xlabel('Time (s)')
            ax.set_title('WTPTA1: Pitch Angle')
            ax.grid(True)

            plt.tight_layout()

            # Save with separate filename
            base, ext = os.path.splitext(filename)
            ren_filename = f'{base}_wt{r+1}{ext}'
            ren_path = os.path.join(output_dir, ren_filename)
            plt.savefig(ren_path, dpi=150)
            print(f"WT3 plot saved: {ren_path}")
            plt.close(fig)
