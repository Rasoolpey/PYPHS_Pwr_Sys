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
