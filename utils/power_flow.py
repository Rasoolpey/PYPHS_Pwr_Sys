"""
Power Flow Solver - General initialization module for finding accurate initial conditions

This module solves the AC power flow equations to determine:
- Bus voltage magnitudes and angles
- Generator power outputs
- Line flows and losses
- Consistent initial states for dynamic simulation
"""
import numpy as np
from scipy.sparse import csr_matrix
from scipy.sparse.linalg import spsolve


class PowerFlowSolver:
    """
    General power flow solver using Newton-Raphson method.
    
    Works with any system configuration from PowerSystemBuilder.
    Supports PQ, PV, and Slack buses.
    """
    
    def __init__(self, builder, coordinator, verbose=True):
        """
        Initialize power flow solver.
        
        Args:
            builder: PowerSystemBuilder instance with system data
            coordinator: PowerSystemCoordinator instance with network data
            verbose: Print initialization details
        """
        self.builder = builder
        self.coordinator = coordinator
        self.system_data = builder.system_data
        
        # Extract network data
        self.n_bus = len(self.system_data.get('Bus', []))
        self.Ybus = coordinator.Ybus_base
        
        # Initialize bus data structures
        self._build_bus_data(verbose)
        
        # Convergence parameters
        self.max_iter = 100  # Increased for robustness with large systems
        self.tol = 1e-4  # Practical convergence tolerance
        
        if verbose:
            print(f"\nPower Flow Solver initialized:")
            print(f"  Total buses: {self.n_bus}")
            print(f"  Slack buses: {len(self.slack_buses)} {list(self.slack_buses)}")
            print(f"  PV buses: {len(self.pv_buses)} {list(self.pv_buses)}")
            print(f"  PQ buses: {len(self.pq_buses)} {list(self.pq_buses)}")
            print(f"\n  Power Specification by Bus:")
            for i in range(self.n_bus):
                if self.P_spec[i] != 0 or self.Q_spec[i] != 0:
                    bus_type = "Slack" if i in self.slack_buses else ("PV" if i in self.pv_buses else "PQ")
                    print(f"    Bus {self.internal_to_bus_idx[i]:2d} ({bus_type:5s}): P={self.P_spec[i]:7.3f}, Q={self.Q_spec[i]:7.3f} pu")
            print(f"\n  Total Power Balance:")
            print(f"    Sum P_spec: {np.sum(self.P_spec):.6f} pu (should be ~0 with slack balancing)")
            print(f"    Sum Q_spec: {np.sum(self.Q_spec):.6f} pu")
    
    def _build_bus_data(self, verbose=True):
        """Build bus classification and initial data from JSON with automatic slack selection."""
        bus_list = self.system_data.get('Bus', [])
        slack_list = self.system_data.get('Slack', [])
        pv_list = self.system_data.get('PV', [])
        pq_list = self.system_data.get('PQ', [])
        genrou_list = self.system_data.get('GENROU', [])
        
        # Create bus index mapping (JSON idx -> array position)
        self.bus_idx_map = {bus['idx']: i for i, bus in enumerate(bus_list)}
        
        # Initialize voltage and power arrays
        self.V = np.ones(self.n_bus)
        self.theta = np.zeros(self.n_bus)
        self.P_spec = np.zeros(self.n_bus)
        self.Q_spec = np.zeros(self.n_bus)
        
        # Load initial voltages from JSON
        for bus in bus_list:
            i = self.bus_idx_map[bus['idx']]
            self.V[i] = bus.get('v0', 1.0)
            self.theta[i] = bus.get('a0', 0.0)
        
        # Determine generator buses
        gen_buses = {gen['bus'] for gen in genrou_list}
        
        # Determine grid/voltage source buses (Slack buses without generators)
        grid_buses = []
        for slack in slack_list:
            bus_idx = slack['bus']
            if bus_idx not in gen_buses:
                grid_buses.append(bus_idx)
        
        # AUTOMATIC SLACK BUS SELECTION STRATEGY
        slack_bus_idx = None
        slack_source = None
        
        # Priority 1: Use explicit Slack from JSON (prefer grids over generators)
        if grid_buses:
            # Use first grid as slack (infinite bus)
            slack_bus_idx = grid_buses[0]
            slack_source = "Grid/Infinite Bus"
            slack_data = next(s for s in slack_list if s['bus'] == slack_bus_idx)
        elif slack_list:
            # Use first slack even if it has a generator
            slack_bus_idx = slack_list[0]['bus']
            slack_source = "Slack Generator"
            slack_data = slack_list[0]
        # Priority 2: If no Slack, use grid sources from builder
        elif hasattr(self.builder, 'grids') and len(self.builder.grids) > 0:
            # Get grid bus from builder
            grid_meta = self.builder.grid_metadata[0]
            slack_bus_idx = grid_meta['bus']
            slack_source = "Grid Voltage Source"
            slack_data = {'bus': slack_bus_idx, 'v0': grid_meta['V_ref'], 'a0': grid_meta['theta_ref']}
        # Priority 3: Select generator with highest capacity as slack
        else:
            # Find generator with highest Sn (power capacity)
            max_capacity = 0
            selected_gen = None
            for gen in genrou_list:
                capacity = gen.get('Sn', 100.0)
                if capacity > max_capacity:
                    max_capacity = capacity
                    selected_gen = gen
            
            if selected_gen:
                slack_bus_idx = selected_gen['bus']
                slack_source = f"Auto-selected Generator (Sn={max_capacity} MVA)"
                # Get voltage from bus data
                bus_data = next(b for b in bus_list if b['idx'] == slack_bus_idx)
                slack_data = {'bus': slack_bus_idx, 'v0': bus_data.get('v0', 1.0), 'a0': bus_data.get('a0', 0.0)}
            else:
                # Fallback: use first bus
                slack_bus_idx = bus_list[0]['idx']
                slack_source = "First Bus (Fallback)"
                slack_data = bus_list[0]
        
        if verbose:
            print(f"\n  Slack Bus Selection: Bus {slack_bus_idx} ({slack_source})")
        
        # Classify buses
        self.slack_buses = []
        self.pv_buses = []
        self.pq_buses = []
        
        # Set slack bus
        i_slack = self.bus_idx_map[slack_bus_idx]
        self.slack_buses.append(i_slack)
        self.V[i_slack] = slack_data.get('v0', 1.0)
        self.theta[i_slack] = slack_data.get('a0', 0.0)
        # Slack power is calculated by solver
        
        # PV buses (generators with voltage control, excluding slack)
        # Collect all generator buses first
        gen_bus_to_pv = {}
        for pv in pv_list:
            gen_bus_to_pv[pv['bus']] = pv
        
        for pv in pv_list:
            bus_idx = pv['bus']
            if bus_idx != slack_bus_idx:
                i = self.bus_idx_map[bus_idx]
                self.pv_buses.append(i)
                # Generator injection is POSITIVE (generation)
                self.P_spec[i] += pv.get('p0', 0.0)  # Add generation
                self.V[i] = pv.get('v0', 1.0)
        
        # Also check for generators not in PV list (from GENROU data)
        for gen in genrou_list:
            bus_idx = gen['bus']
            if bus_idx != slack_bus_idx:
                i = self.bus_idx_map[bus_idx]
                # Only add if not already classified as PV
                if i not in self.pv_buses:
                    # Check if there's PV data for this bus
                    if bus_idx in gen_bus_to_pv:
                        self.pv_buses.append(i)
                        self.P_spec[i] += gen_bus_to_pv[bus_idx].get('p0', 0.7)
                        self.V[i] = gen_bus_to_pv[bus_idx].get('v0', 1.0)
                    else:
                        # No PV data, but generator exists - add as PV with default power
                        self.pv_buses.append(i)
                        self.P_spec[i] += 0.7  # Default 0.7 pu generation
                        self.V[i] = 1.0
        
        # PQ buses (load buses)
        for pq in pq_list:
            bus_idx = pq['bus']
            i = self.bus_idx_map[bus_idx]
            # Only classify as PQ if not already slack or PV
            if i not in self.slack_buses and i not in self.pv_buses:
                if i not in self.pq_buses:
                    self.pq_buses.append(i)
            # Loads are NEGATIVE injections (subtract from bus power)
            self.P_spec[i] -= pq.get('p0', 0.0)
            self.Q_spec[i] -= pq.get('q0', 0.0)
        
        # Remaining buses are PQ with zero injection
        all_classified = set(self.slack_buses + self.pv_buses + self.pq_buses)
        for i in range(self.n_bus):
            if i not in all_classified:
                self.pq_buses.append(i)
        
        # Convert to arrays
        self.slack_buses = np.array(self.slack_buses, dtype=int)
        self.pv_buses = np.array(self.pv_buses, dtype=int)
        self.pq_buses = np.array(self.pq_buses, dtype=int)
        
        # Create internal index to bus idx mapping for diagnostics
        bus_list = self.system_data.get('Bus', [])
        self.internal_to_bus_idx = {}
        for bus in bus_list:
            i = self.bus_idx_map[bus['idx']]
            self.internal_to_bus_idx[i] = bus['idx']
        
        # Validate system
        self._validate_system(verbose)
    
    def _validate_system(self, verbose=True):
        """Validate system setup before solving power flow."""
        issues = []
        
        # Check 1: At least one slack bus must exist
        if len(self.slack_buses) == 0:
            issues.append("No slack bus - power cannot be balanced")
        
        # Check 2: Ybus should be non-singular
        if np.linalg.cond(self.Ybus.todense() if hasattr(self.Ybus, 'todense') else self.Ybus) > 1e12:
            issues.append("Ybus is nearly singular - check for isolated buses")
        
        # Check 3: Check power balance (should be close if system is reasonable)
        total_P = np.sum(self.P_spec)
        total_Q = np.sum(self.Q_spec)
        if abs(total_P) > 10.0:  # Very large imbalance
            issues.append(f"Large power imbalance: P={total_P:.3f} pu")
        
        # Check 4: Ensure all buses are classified
        total_classified = len(self.slack_buses) + len(self.pv_buses) + len(self.pq_buses)
        if total_classified != self.n_bus:
            issues.append(f"Not all buses classified: {total_classified}/{self.n_bus}")
        
        if issues and verbose:
            print("\n  Validation Warnings:")
            for issue in issues:
                print(f"    - {issue}")
        
        return len(issues) == 0
    
    def solve(self, verbose=True):
        """
        Solve AC power flow using Newton-Raphson method.
        
        Args:
            verbose: Print iteration details
        
        Returns:
            converged (bool): True if solution converged
        """
        if verbose:
            print("\nSolving AC Power Flow (Newton-Raphson)...")
        
        # Unknown variables: theta for PQ and PV buses, V for PQ buses
        n_theta = len(self.pq_buses) + len(self.pv_buses)
        n_V = len(self.pq_buses)
        n_unknowns = n_theta + n_V
        
        if n_unknowns == 0:
            if verbose:
                print("  No unknowns - system is fully specified (all slack)")
            # Still calculate power for slack bus
            S_calc = self._calculate_power()
            return True
        
        # Build index mappings for unknowns
        unknown_theta_buses = np.concatenate([self.pv_buses, self.pq_buses])
        unknown_V_buses = self.pq_buses
        
        # Use JSON initial conditions as starting point (better than flat start)
        # Voltages and angles are already loaded from JSON in _build_bus_data
        # Just ensure load buses have reasonable voltages
        for i in unknown_V_buses:
            if self.V[i] == 1.0:  # If not specified in JSON
                self.V[i] = 0.98  # Slightly lower for unspecified load buses
        
        for iteration in range(self.max_iter):
            # Calculate power mismatches
            S_calc = self._calculate_power()
            P_calc = S_calc.real
            Q_calc = S_calc.imag
            
            # Build mismatch vector
            dP = self.P_spec[unknown_theta_buses] - P_calc[unknown_theta_buses]
            dQ = self.Q_spec[unknown_V_buses] - Q_calc[unknown_V_buses]
            mismatch = np.concatenate([dP, dQ])
            
            # Check convergence
            max_mismatch = np.max(np.abs(mismatch))
            if verbose and iteration == 0:
                print(f"  Initial max mismatch: {max_mismatch:.6e}")
            
            if max_mismatch < self.tol:
                if verbose:
                    print(f"  Converged in {iteration} iterations")
                    print(f"  Final max mismatch: {max_mismatch:.6e}")
                return True
            
            # Build Jacobian
            J = self._build_jacobian(unknown_theta_buses, unknown_V_buses)
            
            # Check for singular Jacobian
            if np.linalg.cond(J) > 1e10:
                if verbose:
                    print(f"  Ill-conditioned Jacobian at iteration {iteration}")
                # Try with damped update
                try:
                    dx = np.linalg.lstsq(J, mismatch, rcond=1e-10)[0]
                    dx *= 0.5  # Damping factor
                except:
                    return False
            else:
                # Solve for updates
                try:
                    dx = np.linalg.solve(J, mismatch)
                except np.linalg.LinAlgError:
                    if verbose:
                        print(f"  Singular Jacobian at iteration {iteration}")
                    return False
            
            # Adaptive damping based on iteration and mismatch size
            max_dx_theta = np.max(np.abs(dx[:n_theta])) if n_theta > 0 else 0
            max_dx_V = np.max(np.abs(dx[n_theta:])) if n_V > 0 else 0
            
            # Adaptive damping strategy - more aggressive to speed convergence
            if iteration < 3:
                # Moderate damping in first iterations
                damping = 0.5
            elif max_mismatch > 1.0:
                # Moderate damping for large mismatches
                damping = 0.6
            elif max_mismatch > 0.5:
                # Light damping
                damping = 0.8
            elif max_dx_theta > 0.5 or max_dx_V > 0.2:
                # Damp very large updates
                damping = 0.7
            else:
                # Full Newton steps when reasonably close
                damping = 1.0
            
            if verbose and iteration < 3:
                print(f"  Applying damping factor {damping:.2f} (max_dx_theta={max_dx_theta:.4f}, max_dx_V={max_dx_V:.4f})")
            
            # Update voltages and angles with damping
            self.theta[unknown_theta_buses] += damping * dx[:n_theta]
            if n_V > 0:
                self.V[unknown_V_buses] += damping * dx[n_theta:]
            
            # Limit voltage magnitude for stability
            self.V = np.clip(self.V, 0.5, 1.5)
            
            if verbose and ((iteration + 1) % 5 == 0 or iteration < 3):
                print(f"  Iteration {iteration + 1}: max mismatch = {max_mismatch:.6e}")
        
        # Check for practical convergence (within 10x tolerance)
        if max_mismatch < self.tol * 10:
            if verbose:
                print(f"  Practical convergence after {self.max_iter} iterations")
                print(f"  Final max mismatch: {max_mismatch:.6e} (within 10x tolerance)")
                print(f"  Solution is acceptable for initialization")
            return True
        
        if verbose:
            print(f"  Failed to converge after {self.max_iter} iterations")
            print(f"  Final max mismatch: {max_mismatch:.6e}")
            print(f"  Note: This may indicate:")
            print(f"    - System is near voltage collapse")
            print(f"    - Incorrect network data or parameters")
            print(f"    - Heavily loaded system requiring better initial guess")
        return False
    
    def _calculate_power(self):
        """Calculate power injections at all buses."""
        V_complex = self.V * np.exp(1j * self.theta)
        I = self.Ybus @ V_complex
        S = V_complex * np.conj(I)
        return S
    
    def _build_jacobian(self, unknown_theta_buses, unknown_V_buses):
        """Build Jacobian matrix for Newton-Raphson."""
        n_theta = len(unknown_theta_buses)
        n_V = len(unknown_V_buses)
        n = n_theta + n_V
        
        J = np.zeros((n, n))
        
        V_complex = self.V * np.exp(1j * self.theta)
        
        # Calculate partial derivatives
        for idx_i, i in enumerate(unknown_theta_buses):
            for idx_j, j in enumerate(unknown_theta_buses):
                # dP/dtheta
                if i == j:
                    # Diagonal: sum of off-diagonal terms
                    val = 0.0
                    for k in range(self.n_bus):
                        if k != i:
                            Yik = self.Ybus[i, k]
                            val += self.V[i] * self.V[k] * np.abs(Yik) * np.sin(
                                self.theta[i] - self.theta[k] - np.angle(Yik)
                            )
                    J[idx_i, idx_j] = -val
                else:
                    # Off-diagonal
                    Yij = self.Ybus[i, j]
                    J[idx_i, idx_j] = self.V[i] * self.V[j] * np.abs(Yij) * np.sin(
                        self.theta[i] - self.theta[j] - np.angle(Yij)
                    )
        
        # dP/dV for PQ buses
        for idx_i, i in enumerate(unknown_theta_buses):
            for idx_j, j in enumerate(unknown_V_buses):
                if i == j:
                    # Diagonal
                    Yii = self.Ybus[i, i]
                    val = self.V[i] * np.abs(Yii) * np.cos(np.angle(Yii))
                    for k in range(self.n_bus):
                        if k != i:
                            Yik = self.Ybus[i, k]
                            val += self.V[k] * np.abs(Yik) * np.cos(
                                self.theta[i] - self.theta[k] - np.angle(Yik)
                            )
                    J[idx_i, n_theta + idx_j] = val
                else:
                    # Off-diagonal
                    Yij = self.Ybus[i, j]
                    J[idx_i, n_theta + idx_j] = self.V[i] * np.abs(Yij) * np.cos(
                        self.theta[i] - self.theta[j] - np.angle(Yij)
                    )
        
        # dQ/dtheta for PQ buses
        for idx_i, i in enumerate(unknown_V_buses):
            for idx_j, j in enumerate(unknown_theta_buses):
                if i == j:
                    # Diagonal
                    val = 0.0
                    for k in range(self.n_bus):
                        if k != i:
                            Yik = self.Ybus[i, k]
                            val -= self.V[i] * self.V[k] * np.abs(Yik) * np.cos(
                                self.theta[i] - self.theta[k] - np.angle(Yik)
                            )
                    J[n_theta + idx_i, idx_j] = -val
                else:
                    # Off-diagonal
                    Yij = self.Ybus[i, j]
                    J[n_theta + idx_i, idx_j] = -self.V[i] * self.V[j] * np.abs(Yij) * np.cos(
                        self.theta[i] - self.theta[j] - np.angle(Yij)
                    )
        
        # dQ/dV for PQ buses
        for idx_i, i in enumerate(unknown_V_buses):
            for idx_j, j in enumerate(unknown_V_buses):
                if i == j:
                    # Diagonal
                    Yii = self.Ybus[i, i]
                    val = -self.V[i] * np.abs(Yii) * np.sin(np.angle(Yii))
                    for k in range(self.n_bus):
                        if k != i:
                            Yik = self.Ybus[i, k]
                            val -= self.V[k] * np.abs(Yik) * np.sin(
                                self.theta[i] - self.theta[k] - np.angle(Yik)
                            )
                    J[n_theta + idx_i, n_theta + idx_j] = val
                else:
                    # Off-diagonal
                    Yij = self.Ybus[i, j]
                    J[n_theta + idx_i, n_theta + idx_j] = -self.V[i] * np.abs(Yij) * np.sin(
                        self.theta[i] - self.theta[j] - np.angle(Yij)
                    )
        
        return J
    
    def get_results(self):
        """
        Get power flow results.
        
        Returns:
            dict with voltage magnitudes, angles, and power injections
        """
        S = self._calculate_power()
        P = S.real
        Q = S.imag
        
        return {
            'V': self.V.copy(),
            'theta': self.theta.copy(),
            'P': P.copy(),
            'Q': Q.copy()
        }
    
    def print_results(self):
        """Print power flow results in tabular format."""
        results = self.get_results()
        bus_list = self.system_data.get('Bus', [])
        
        print("\n" + "="*80)
        print("                    POWER FLOW RESULTS")
        print("="*80)
        print(f"{'Bus':>4} {'Type':>6} {'V (pu)':>10} {'Angle (deg)':>12} {'P (pu)':>12} {'Q (pu)':>12}")
        print("-"*80)
        
        for bus in bus_list:
            i = self.bus_idx_map[bus['idx']]
            
            # Determine bus type
            if i in self.slack_buses:
                bus_type = "Slack"
            elif i in self.pv_buses:
                bus_type = "PV"
            else:
                bus_type = "PQ"
            
            print(f"{bus['idx']:>4} {bus_type:>6} {results['V'][i]:>10.6f} "
                  f"{np.degrees(results['theta'][i]):>12.4f} "
                  f"{results['P'][i]:>12.6f} {results['Q'][i]:>12.6f}")
        
        print("-"*80)
        print(f"{'Total':>23} {np.sum(results['P']):>12.6f} {np.sum(results['Q']):>12.6f}")
        print("="*80)
    
    def update_system_data(self):
        """
        Update system JSON data with power flow results.
        
        This updates:
        - Bus voltages and angles
        - Generator power outputs
        - Initial conditions for dynamic simulation
        """
        results = self.get_results()
        
        print("\nUpdating system data with power flow results...")
        
        # Update Bus data
        for bus in self.system_data.get('Bus', []):
            i = self.bus_idx_map[bus['idx']]
            bus['v0'] = float(results['V'][i])
            bus['a0'] = float(results['theta'][i])
        
        # Update Slack data
        for slack in self.system_data.get('Slack', []):
            i = self.bus_idx_map[slack['bus']]
            slack['p0'] = float(results['P'][i])
            slack['q0'] = float(results['Q'][i])
            slack['v0'] = float(results['V'][i])
            slack['a0'] = float(results['theta'][i])
        
        # Update PV data
        for pv in self.system_data.get('PV', []):
            i = self.bus_idx_map[pv['bus']]
            # P is already specified, update Q
            pv['q0'] = float(results['Q'][i])
            pv['v0'] = float(results['V'][i])
        
        print("  System data updated successfully")


def run_power_flow(builder, coordinator, verbose=True):
    """
    Convenience function to run power flow and update system.
    
    Args:
        builder: PowerSystemBuilder instance
        coordinator: PowerSystemCoordinator instance
        verbose: Print detailed output
    
    Returns:
        PowerFlowSolver instance with results (or None if failed)
    """
    pf = PowerFlowSolver(builder, coordinator, verbose=verbose)
    converged = pf.solve(verbose=verbose)
    
    if converged:
        if verbose:
            pf.print_results()
        pf.update_system_data()
        return pf
    else:
        if verbose:
            print("\n" + "!"*70)
            print("  ERROR: Power flow did not converge!")
            print("  This indicates a fundamental problem with system setup:")
            print("    - Check that loads and generation are balanced")
            print("    - Verify network impedances are correct")
            print("    - Ensure at least one slack bus exists")
            print("    - Check for isolated buses or islands")
            print("!"*70)
        raise RuntimeError(
            "Power flow failed to converge. Cannot proceed with inaccurate initial conditions. "
            "Please check system configuration in JSON file."
        )


def build_initial_state_vector(builder, coordinator, verbose=True):
    """
    COMPLETE GENERAL INITIAL CONDITION CALCULATOR FOR ANY POWER SYSTEM.
    
    This function handles ALL initialization - works with any component types:
    - Synchronous generators with exciters and governors
    - Inverter-based resources
    - Grid/slack buses
    - Any other future component types
    
    This is the ONLY place where initial conditions are calculated.
    The simulator just uses the returned state vector.
    
    Args:
        builder: PowerSystemBuilder instance (contains all components)
        coordinator: PowerSystemCoordinator instance (contains network data)
        verbose: Print detailed output
    
    Returns:
        x0: Complete initial state vector ready for simulation (numpy array)
    """
    if verbose:
        print("\n" + "="*80)
        print("  BUILDING INITIAL STATE VECTOR FROM POWER FLOW")
        print("="*80)
    
    # Get power flow results (already in system_data after power flow solve)
    system_data = builder.system_data
    gen_to_bus = coordinator.gen_to_bus
    
    # Extract bus data
    bus_data = {b['idx']: b for b in system_data.get('Bus', [])}
    slack_data = {s['bus']: s for s in system_data.get('Slack', [])}
    pv_data = {p['bus']: p for p in system_data.get('PV', [])}
    
    # Detect what components exist in this system
    n_gen = len(builder.generators) if hasattr(builder, 'generators') else 0
    n_grid = len(builder.grids) if hasattr(builder, 'grids') else 0
    
    if verbose:
        print(f"\nSystem components detected:")
        print(f"  Generators: {n_gen}")
        print(f"  Grids: {n_grid}")
    
    # ========================================================================
    # PART 1: EXTRACT POWER FLOW RESULTS
    # ========================================================================
    if verbose and n_gen > 0:
        print(f"\nExtracting power flow results...")
    
    V_mag = np.zeros(n_gen)
    V_ang = np.zeros(n_gen)
    P_gen = np.zeros(n_gen)
    Q_gen = np.zeros(n_gen)
    
    # Identify slack generators for special handling
    slack_gen_indices = []
    
    for i in range(n_gen):
        bus_idx = gen_to_bus[i]
        
        # Check if this generator is at a slack bus
        if bus_idx in slack_data:
            slack_gen_indices.append(i)
        
        # Terminal voltage from power flow
        if bus_idx in bus_data:
            V_mag[i] = bus_data[bus_idx].get('v0', 1.0)
            V_ang[i] = bus_data[bus_idx].get('a0', 0.0)
        else:
            V_mag[i] = 1.0
            V_ang[i] = 0.0
        
        # Power from power flow
        if bus_idx in slack_data:
            P_gen[i] = slack_data[bus_idx].get('p0', 0.0)
            Q_gen[i] = slack_data[bus_idx].get('q0', 0.0)
        elif bus_idx in pv_data:
            P_gen[i] = pv_data[bus_idx].get('p0', 0.0)
            Q_gen[i] = pv_data[bus_idx].get('q0', 0.0)
        else:
            P_gen[i] = 0.7  # Default
            Q_gen[i] = 0.0
    
    if verbose and len(slack_gen_indices) > 0:
        print(f"\n  Slack generators detected: {slack_gen_indices}")
        print(f"  (Will use conservative limits for exciter/governor initialization)")
    
    # ========================================================================
    # PART 2: CALCULATE GENERATOR INTERNAL STATES
    # ========================================================================
    delta_vals = np.zeros(n_gen)
    psi_f_vals = np.zeros(n_gen)
    Vd = np.zeros(n_gen)
    Vq = np.zeros(n_gen)
    Id = np.zeros(n_gen)
    Iq = np.zeros(n_gen)
    
    if n_gen > 0 and verbose:
        print(f"\nCalculating generator internal states...")
    
    for i in range(n_gen):
        meta = builder.gen_metadata[i]
        
        # Generator parameters
        Xd2 = meta.get('xd2', 0.0278)
        ra = meta.get('ra', 0.0)
        
        # Phasor calculations
        V_phasor = V_mag[i] * np.exp(1j * V_ang[i])
        S = P_gen[i] + 1j * Q_gen[i]
        I_phasor = np.conj(S / V_phasor)
        
        # Internal voltage: E'' = V + (ra + jXd'') * I
        Z_gen = ra + 1j * Xd2
        E_phasor = V_phasor + Z_gen * I_phasor
        
        # Rotor angle and field flux
        delta_vals[i] = np.angle(E_phasor)
        E_mag = np.abs(E_phasor)
        gd1 = meta.get('gd1', 0.79)
        psi_f_vals[i] = E_mag / gd1
        
        # Transform to dq frame
        delta = delta_vals[i]
        V_R, V_I = np.real(V_phasor), np.imag(V_phasor)
        I_R, I_I = np.real(I_phasor), np.imag(I_phasor)
        
        Vd[i] = V_R * np.sin(delta) - V_I * np.cos(delta)
        Vq[i] = V_R * np.cos(delta) + V_I * np.sin(delta)
        Id[i] = I_R * np.sin(delta) - I_I * np.cos(delta)
        Iq[i] = I_R * np.cos(delta) + I_I * np.sin(delta)
    
    # ========================================================================
    # PART 3: BUILD COMPLETE STATE VECTOR FOR ALL COMPONENTS
    # ========================================================================
    if verbose:
        print(f"\nBuilding state vector for all components...")
    
    # Build state index map for heterogeneous components (same as fault_sim_modular.py)
    gen_state_counts = [gen.n_states for gen in builder.generators] if n_gen > 0 else []
    exc_state_counts = [exc.n_states for exc in builder.exciters] if n_gen > 0 else []
    gov_state_counts = [gov.n_states for gov in builder.governors] if n_gen > 0 else []
    
    machine_state_lists = []  # List of state vectors, one per machine (generic)
    grid_state_lists = []  # List of state vectors, one per grid
    
    # --- Initialize Each Generator's States (fully generic) ---
    for i in range(n_gen):
        meta = builder.gen_metadata[i]
        omega_b = meta['omega_b']
        M = meta['M']
        ra = meta['ra']
        
        # Allocate state vector for this machine (generic sizes)
        n_gen_states = gen_state_counts[i]
        n_exc_states = exc_state_counts[i]
        n_gov_states = gov_state_counts[i]
        machine_states = []
        
        # Core generator states (always 7 for GENROU - but flexible)
        psi_d = (Vq[i] + ra * Iq[i]) / omega_b
        psi_q = -(Vd[i] + ra * Id[i]) / omega_b
        
        gen_x0 = np.zeros(n_gen_states)
        gen_x0[0] = delta_vals[i]  # delta
        gen_x0[1] = M * 1.0         # p = M * omega (omega=1 at equilibrium)
        if n_gen_states > 2:
            gen_x0[2] = psi_d       # psi_d
        if n_gen_states > 3:
            gen_x0[3] = psi_q       # psi_q
        if n_gen_states > 4:
            gen_x0[4] = psi_f_vals[i]  # psi_f
        if n_gen_states > 5:
            gen_x0[5] = 0.0         # psi_kd  
        if n_gen_states > 6:
            gen_x0[6] = 0.0         # psi_kq
        
        machine_states.append(gen_x0)
        
        # --- Initialize Exciter States (generic for any exciter model) ---
        if n_exc_states > 0:
            exc_core = builder.exciters[i]
            exc_meta = builder.exc_metadata[i]
            
            # Calculate target field voltage
            xd = meta['xd']
            xl = meta['xl']
            xd1 = meta['xd1']
            Xad = xd - xl
            Xfl = (Xad * (xd1 - xl)) / (Xad - (xd1 - xl))
            Lf = Xad + Xfl
            Kfd_scale = Lf * 2.0
            Efd_raw = psi_f_vals[i] / Kfd_scale
            
            # Use calculated Efd directly - no hardcoded clipping
            # The Efd value is derived from power flow and machine parameters
            # and must be consistent regardless of system base or machine rating
            Efd_target = Efd_raw

            # Only warn if Efd is negative (physically unreasonable)
            if Efd_target < 0:
                if verbose:
                    gen_type = "SLACK" if i in slack_gen_indices else "PV"
                    print(f"  Gen {i} ({gen_type}): WARNING - negative Efd={Efd_target:.3f}, clamping to 0.1")
                Efd_target = 0.1
            
            if verbose:
                gen_type = "SLACK" if i in slack_gen_indices else "PV"
                print(f"  Gen {i} ({gen_type}): psi_f={psi_f_vals[i]:.3f}, Efd_target={Efd_target:.3f}")
            
            # GENERIC initialization - call component's init_fn (NO hardcoded model logic)
            if exc_core.init_fn is not None:
                exc_x0 = exc_core.init_fn(Efd_eq=Efd_target, V_mag=V_mag[i])
            else:
                # Fallback if init_fn not provided (should not happen with proper components)
                exc_x0 = np.zeros(n_exc_states)
                exc_x0[0] = V_mag[i]
                if n_exc_states > 1:
                    exc_x0[1] = Efd_target
            
            machine_states.append(exc_x0)
        else:
            # No exciter - skip this component
            pass
        
        # --- Initialize Governor States (generic for any governor model) ---
        if n_gov_states > 0:
            
            # Mechanical power = electrical power at equilibrium
            # Pm must match Pe exactly for zero initial derivatives
            Pm_raw = Vd[i] * Id[i] + Vq[i] * Iq[i]

            # Use calculated Pm directly - no hardcoded clipping
            # The value depends on system base and machine rating
            Pm_eq = Pm_raw

            # Only warn if Pm is negative (physically unreasonable for a generator)
            if Pm_eq < 0:
                if verbose:
                    gen_type = "SLACK" if i in slack_gen_indices else "PV"
                    print(f"  Gen {i} ({gen_type}): WARNING - negative Pm={Pm_eq:.3f}, clamping to 0.0")
                Pm_eq = 0.0
            
            # Governor initialization (fully generic using init_fn)
            if hasattr(builder, 'governors') and i < len(builder.governors):
                gov_core = builder.governors[i]
                
                if gov_core.init_fn is not None:
                    gov_x0 = gov_core.init_fn(Pm_eq=Pm_eq)
                else:
                    # Fallback: all governor states = Pm_eq
                    gov_x0 = np.zeros(n_gov_states)
                    for j in range(n_gov_states):
                        gov_x0[j] = Pm_eq
                
                machine_states.append(gov_x0)
                
                # Update governor reference
                builder.gov_metadata[i]['Pref'] = Pm_eq
                builder.gov_metadata[i]['wref'] = 1.0
        else:
            # No governor - skip this component
            pass
        
        # Concatenate all states for this machine and add to list
        machine_x0 = np.concatenate(machine_states)
        machine_state_lists.append(machine_x0)
    
    # --- Initialize Grid/Slack Bus States (generic) ---
    for i in range(n_grid):
        grid_meta = builder.grid_metadata[i]
        grid_x0 = np.array([grid_meta['V_ref'], grid_meta['theta_ref']])
        grid_state_lists.append(grid_x0)
    
    # ========================================================================
    # PART 4: NETWORK CONSISTENCY CHECK & ADJUSTMENT
    # ========================================================================
    if n_gen > 0:
        if verbose:
            print(f"\nAdjusting for network consistency...")

        # Extract generator states for network solver (first 7 states of each machine)
        gen_states = []
        for machine_x0 in machine_state_lists:
            # Extract first 7 (or fewer) states for generator
            gen_x = machine_x0[:min(7, len(machine_x0))]
            # Pad to 7 if needed (though GENROU always has 7)
            if len(gen_x) < 7:
                gen_x = np.concatenate([gen_x, np.zeros(7 - len(gen_x))])
            gen_states.append(gen_x)
        gen_states = np.array(gen_states)
        
        # Get grid voltages
        grid_voltages = None
        if n_grid > 0:
            grid_voltages = np.array([grid_x[0] * np.exp(1j * grid_x[1])
                                      for grid_x in grid_state_lists])

        Id_net, Iq_net, Vd_net, Vq_net = coordinator.solve_network(gen_states, grid_voltages=grid_voltages)
        P_elec = Vd_net * Id_net + Vq_net * Iq_net

        # Adjust stator fluxes to match actual network voltages/currents
        # At equilibrium: psi_d = (Vq + ra*Iq)/omega_b, psi_q = -(Vd + ra*Id)/omega_b
        for i in range(n_gen):
            meta = builder.gen_metadata[i]
            omega_b = meta['omega_b']
            ra = meta['ra']

            psi_d_new = (Vq_net[i] + ra * Iq_net[i]) / omega_b
            psi_q_new = -(Vd_net[i] + ra * Id_net[i]) / omega_b

            # Update machine state list (generic - works for any state count)
            machine_x = machine_state_lists[i]
            if len(machine_x) > 2:
                psi_d_old = machine_x[2]
            if len(machine_x) > 3:
                psi_q_old = machine_x[3]
                
                if verbose and abs(psi_d_new - psi_d_old) > 1e-4:
                    print(f"  Gen {i}: psi_d adjusted {psi_d_old:.4f} -> {psi_d_new:.4f}, "
                          f"psi_q adjusted {psi_q_old:.4f} -> {psi_q_new:.4f}")
                
                machine_x[2] = psi_d_new
                machine_x[3] = psi_q_new

        # Refine exciter states using actual network quantities (FULLY GENERIC)
        for i in range(n_gen):
            if exc_state_counts[i] > 0:
                exc_core = builder.exciters[i]
                exc_meta = builder.exc_metadata[i]
                gen_meta = builder.gen_metadata[i]
                
                # Compute actual terminal voltage magnitude from network solution
                Vt_actual = np.sqrt(Vd_net[i]**2 + Vq_net[i]**2)
                
                # Recompute Efd_target from actual psi_f (may have been adjusted)
                xd = gen_meta['xd']
                xl = gen_meta['xl']
                xd1 = gen_meta['xd1']
                Xad = xd - xl
                Xfl = (Xad * (xd1 - xl)) / (Xad - (xd1 - xl))
                Lf = Xad + Xfl
                Kfd_scale = Lf * 2.0
                
                # Calculate state offsets within this machine's state vector
                n_gen_states_i = gen_state_counts[i]
                exc_offset = n_gen_states_i
                
                machine_x = machine_state_lists[i]
                psi_f_actual = machine_x[4]  # Generator state 4
                Efd_actual = psi_f_actual / Kfd_scale
                
                # Re-initialize exciter with actual network quantities (GENERIC)
                if exc_core.init_fn is not None:
                    exc_x_refined = exc_core.init_fn(
                        Efd_eq=Efd_actual,
                        V_mag=Vt_actual,
                        Vd=Vd_net[i],
                        Vq=Vq_net[i],
                        Id=Id_net[i],
                        Iq=Iq_net[i],
                        psi_f=psi_f_actual
                    )
                    # Update exciter states
                    for j in range(exc_state_counts[i]):
                        machine_x[exc_offset + j] = exc_x_refined[j]
                    
                    if verbose:
                        print(f"  Gen {i}: {exc_core.model_name} refined with actual network V/I")
                else:
                    # Fallback: just update voltage measurement
                    machine_x[exc_offset + 0] = Vt_actual



        # Adjust mechanical power to match electrical power (generic for heterogeneous governors)
        for i in range(n_gen):
            if gov_state_counts[i] > 0:
                n_gen_states_i = gen_state_counts[i]
                n_exc_states_i = exc_state_counts[i]
                gov_offset = n_gen_states_i + n_exc_states_i
                
                machine_x = machine_state_lists[i]
                gov_core = builder.governors[i]
                gov_meta = builder.gov_metadata[i]
                
                Pm_adjusted = P_elec[i]

                # Only clamp negative values (physically unreasonable)
                if Pm_adjusted < 0:
                    if verbose:
                        gen_type = "SLACK" if i in slack_gen_indices else "PV"
                        print(f"  Gen {i} ({gen_type}): WARNING - negative network Pm={Pm_adjusted:.3f}, clamping to 0.0")
                    Pm_adjusted = 0.0
                
                # Generic governor adjustment: re-initialize with adjusted Pm
                if gov_core.init_fn is not None:
                    gov_x_adjusted = gov_core.init_fn(Pm_eq=Pm_adjusted)
                    for j in range(gov_state_counts[i]):
                        machine_x[gov_offset + j] = gov_x_adjusted[j]
                else:
                    # Fallback: set all states to Pm_adjusted
                    for j in range(gov_state_counts[i]):
                        machine_x[gov_offset + j] = Pm_adjusted
                
                builder.gov_metadata[i]['Pref'] = Pm_adjusted
    
    # ========================================================================
    # PART 5: ASSEMBLE FINAL STATE VECTOR (fully generic)
    # ========================================================================
    # Concatenate all machine state vectors and grid state vectors
    all_states = machine_state_lists + grid_state_lists
    x0 = np.concatenate(all_states) if all_states else np.array([])
    
    if verbose:
        print(f"\nInitial state vector built:")
        print(f"  Total states: {len(x0)}")
        if n_gen > 0:
            total_machine_states = sum(len(m) for m in machine_state_lists)
            print(f"  Machine states: {total_machine_states}")
            print(f"    Per-machine state counts: Gen={gen_state_counts}, Exc={exc_state_counts}, Gov={gov_state_counts}")
        if n_grid > 0:
            total_grid_states = sum(len(g) for g in grid_state_lists)
            print(f"  Grid states: {total_grid_states}")
        print("\n" + "="*80)
        print("  INITIAL CONDITIONS COMPLETE")
        print("="*80)
    
    return x0
