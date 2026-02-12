"""
Numerical equilibrium solver for power system dynamics.

Finds true equilibrium where all dx/dt = 0 by numerically solving
the nonlinear equilibrium equations.
"""
import numpy as np
from scipy.optimize import fsolve, least_squares


def find_numerical_equilibrium(simulator, x0_initial, max_iter=50, tol=1e-6, verbose=True):
    """
    Find numerical equilibrium starting from initial guess.
    
    Strategy:
    1. Fix generator mechanical states (delta, omega, fluxes) - determined by network
    2. Solve for exciter and governor states that zero their derivatives
    3. Use scipy optimization to find roots of dynamics equations
    
    Args:
        simulator: ModularFaultSimulator instance
        x0_initial: Initial state guess (from algebraic equilibrium)
        max_iter: Maximum iterations
        tol: Tolerance for convergence (max |dx/dt|)
        verbose: Print progress
    
    Returns:
        x0_equilibrium: Equilibrium state vector
        converged: True if converged
    """
    if verbose:
        print("\n" + "="*70)
        print("  NUMERICAL EQUILIBRIUM SOLVER")
        print("="*70)
    
    # Reshape to get per-machine states
    gen_size = simulator.n_gen * simulator.states_per_machine
    x0_gen = x0_initial[:gen_size].reshape(simulator.n_gen, simulator.states_per_machine)
    x0_grid = x0_initial[gen_size:]
    
    # Identify which states to optimize
    # Generator states: [delta, omega, psi_d, psi_q, psi_fd, psi_kd, psi_kq]
    # Exciter states: varies by model
    # Governor states: varies by model
    
    # We'll optimize exciter and governor states while holding generator states fixed
    n_gen_states = simulator.n_gen_states
    n_exc_states = simulator.n_exc_states
    n_gov_states = simulator.n_gov_states
    
    # Build lists of which states are free vs fixed
    free_state_indices = []
    fixed_state_values = []
    
    for i in range(simulator.n_gen):
        base_idx = i * simulator.states_per_machine
        
        # Fix generator mechanical states (0-6)
        for j in range(n_gen_states):
            fixed_state_values.append((base_idx + j, x0_gen[i, j]))
        
        # Free exciter states (7 to 7+n_exc-1)
        exc_start = n_gen_states
        for j in range(n_exc_states):
            free_state_indices.append(base_idx + exc_start + j)
        
        # Free governor states
        gov_start = exc_start + n_exc_states
        for j in range(n_gov_states):
            free_state_indices.append(base_idx + gov_start + j)
    
    n_free = len(free_state_indices)
    
    if verbose:
        print(f"\nOptimization problem:")
        print(f"  Total states: {len(x0_initial)}")
        print(f"  Fixed (generator): {len(fixed_state_values)}")
        print(f"  Free (exciter+governor): {n_free}")
        print(f"  Grid states: {len(x0_grid)}")
    
    # Extract initial guess for free states
    x_free_init = x0_initial[free_state_indices]
    
    # Define residual function: we want dx/dt = 0 for free states
    def residual(x_free):
        """Compute derivatives of free states."""
        # Reconstruct full state vector
        x_full = x0_initial.copy()
        x_full[free_state_indices] = x_free
        
        # Evaluate dynamics
        dx = simulator.dynamics(0.0, x_full)
        
        # Return derivatives of free states
        return dx[free_state_indices]
    
    if verbose:
        # Check initial residual
        res_init = residual(x_free_init)
        max_res_init = np.max(np.abs(res_init))
        print(f"\nInitial residual: max|dx/dt| = {max_res_init:.3e}")
    
    # Solve using least squares (more robust than fsolve)
    if verbose:
        print(f"\nSolving equilibrium equations...")
    
    result = least_squares(
        residual,
        x_free_init,
        method='lm',  # Levenberg-Marquardt
        ftol=tol,
        xtol=tol,
        max_nfev=max_iter * n_free,
        verbose=2 if verbose else 0
    )
    
    # Reconstruct equilibrium state
    x_equilibrium = x0_initial.copy()
    x_equilibrium[free_state_indices] = result.x
    
    # Check final residual
    dx_final = simulator.dynamics(0.0, x_equilibrium)
    max_derivative = np.max(np.abs(dx_final))
    
    converged = result.success and max_derivative < tol * 10
    
    if verbose:
        print(f"\n" + "="*70)
        print(f"  RESULTS")
        print("="*70)
        print(f"  Converged: {converged}")
        print(f"  Iterations: {result.nfev // n_free}")
        print(f"  Final max |dx/dt|: {max_derivative:.3e}")
        
        if converged:
            print(f"  Status: TRUE EQUILIBRIUM FOUND!")
        else:
            print(f"  Status: Did not fully converge")
            print(f"  Message: {result.message}")
        print("="*70 + "\n")
    
    return x_equilibrium, converged


def find_equilibrium_adaptive(simulator, x0_initial, tol=1e-5, max_time=10.0, 
                             check_interval=0.5, verbose=True):
    """
    Find equilibrium by simulating until derivatives converge to near-zero.
    
    Adaptively monitors state changes and stops when equilibrium is reached.
    
    Args:
        simulator: ModularFaultSimulator instance
        x0_initial: Initial state guess
        tol: Convergence tolerance for max|dx/dt|
        max_time: Maximum simulation time (safety limit)
        check_interval: Time between convergence checks (seconds)
        verbose: Print progress
    
    Returns:
        x0_equilibrium: Equilibrium state vector
        converged: True if converged within tolerance
    """
    if verbose:
        print("\n" + "="*70)
        print("  ADAPTIVE EQUILIBRIUM FINDER")
        print("="*70)
        print(f"  Tolerance: max|dx/dt| < {tol:.1e}")
        print(f"  Max time: {max_time}s")
    
    from scipy.integrate import solve_ivp
    
    t_current = 0.0
    x_current = x0_initial.copy()
    iteration = 0
    
    while t_current < max_time:
        iteration += 1
        t_next = min(t_current + check_interval, max_time)
        
        # Simulate for one interval
        sol = solve_ivp(
            simulator.dynamics,
            [t_current, t_next],
            x_current,
            method='Radau',  # Good for stiff systems
            rtol=1e-8,
            atol=1e-10,
            dense_output=False
        )
        
        x_next = sol.y[:, -1]
        
        # Check convergence: both derivatives and state changes should be small
        dx = simulator.dynamics(t_next, x_next)
        max_derivative = np.max(np.abs(dx))
        
        # Also check state change rate
        state_change = np.max(np.abs(x_next - x_current)) / check_interval
        
        if verbose and (iteration % 5 == 1 or max_derivative < tol):
            print(f"  t={t_next:.2f}s: max|dx/dt|={max_derivative:.3e}, max|delta_x/dt|={state_change:.3e}")
        
        # Check convergence
        if max_derivative < tol and state_change < tol:
            if verbose:
                print(f"\n  [OK] Converged at t={t_next:.2f}s")
                print(f"       Final max|dx/dt| = {max_derivative:.3e}")
                print("="*70 + "\n")
            return x_next, True
        
        # Continue from current state
        x_current = x_next
        t_current = t_next
    
    # Reached max time without full convergence
    dx_final = simulator.dynamics(max_time, x_current)
    max_deriv_final = np.max(np.abs(dx_final))
    
    if verbose:
        print(f"\n  [!] Reached max time ({max_time}s)")
        print(f"      Final max|dx/dt| = {max_deriv_final:.3e}")
        if max_deriv_final < tol * 10:
            print(f"      Close to equilibrium - acceptable")
        else:
            print(f"      Did not fully converge")
        print("="*70 + "\n")
    
    return x_current, (max_deriv_final < tol * 10)


def find_equilibrium_simple(simulator, x0_initial, settling_time=2.0, verbose=True):
    """
    Find equilibrium by simulating for a short time to let fast dynamics settle.
    
    This is a simpler alternative for quick initialization.
    
    Args:
        simulator: ModularFaultSimulator instance
        x0_initial: Initial state guess
        settling_time: Time to simulate (seconds)
        verbose: Print progress
    
    Returns:
        x0_equilibrium: Settled state vector
    """
    if verbose:
        print("\n" + "="*70)
        print("  EQUILIBRIUM VIA SHORT SETTLING SIMULATION")
        print("="*70)
        print(f"  Simulating {settling_time}s to let fast dynamics settle...")
    
    from scipy.integrate import solve_ivp
    
    # Simulate for short time with tight tolerances
    sol = solve_ivp(
        simulator.dynamics,
        [0, settling_time],
        x0_initial,
        method='Radau',  # Good for stiff systems
        rtol=1e-8,
        atol=1e-10,
        dense_output=False
    )
    
    # Take final state as equilibrium
    x_equilibrium = sol.y[:, -1]
    
    # Check derivatives at final state
    dx_final = simulator.dynamics(settling_time, x_equilibrium)
    max_derivative = np.max(np.abs(dx_final))
    
    if verbose:
        print(f"  Final max |dx/dt|: {max_derivative:.3e}")
        if max_derivative < 1e-4:
            print(f"  Status: Good equilibrium achieved")
        else:
            print(f"  Status: May need longer settling time")
        print("="*70 + "\n")
    
    return x_equilibrium
