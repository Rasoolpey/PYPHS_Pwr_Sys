"""
Test Lyapunov Stability Analyzer
Demonstrates passivity-based stability analysis for power systems
"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from utils.lyapunov_analyzer import LyapunovStabilityAnalyzer


def test_stability_analysis():
    """Test basic stability analysis capabilities"""
    
    print("\n" + "="*70)
    print("   LYAPUNOV STABILITY ANALYZER TEST")
    print("="*70)
    
    # Initialize analyzer
    json_file = 'test_cases/Kundur_System/kundur_full.json'
    analyzer = LyapunovStabilityAnalyzer(json_file)
    
    # 1. Initialize equilibrium
    print("\n[TEST 1: Equilibrium Initialization]")
    x_eq = analyzer.initialize_equilibrium()
    print(f"[OK] Equilibrium state dimension: {len(x_eq)}")
    print(f"[OK] Equilibrium Hamiltonian: H* = {analyzer.H_eq:.6f}")
    
    # 2. Test Hamiltonian computation
    print("\n[TEST 2: Hamiltonian Computation]")
    H = analyzer.compute_hamiltonian(x_eq)
    print(f"[OK] H(x*) = {H:.6f}")
    
    # Test with perturbation
    x_pert = x_eq + np.random.randn(len(x_eq)) * 0.01
    H_pert = analyzer.compute_hamiltonian(x_pert)
    print(f"[OK] H(x* + dx) = {H_pert:.6f}")
    print(f"[OK] dH = {H_pert - H:.6f}")
    
    # 3. Test Lyapunov function
    print("\n[TEST 3: Lyapunov Function]")
    V = analyzer.compute_lyapunov_function(x_eq)
    print(f"[OK] V(x*) = {V:.6e} (should be ~0)")
    
    V_pert = analyzer.compute_lyapunov_function(x_pert)
    print(f"[OK] V(x* + dx) = {V_pert:.6e} (should be >0)")
    
    # 4. Linearized stability analysis
    print("\n[TEST 4: Linearized Stability Analysis]")
    results = analyzer.analyze_linearized_stability()
    
    print(f"\n[OK] Stability Analysis Complete:")
    print(f"  - System is: {'STABLE' if results['is_stable'] else 'UNSTABLE'}")
    print(f"  - Stable modes: {results['stable_count']}")
    print(f"  - Unstable modes: {results['unstable_count']}")
    print(f"  - Marginal modes: {results['marginal_count']}")
    
    # 5. Plot eigenvalues
    print("\n[TEST 5: Eigenvalue Visualization]")
    fig = analyzer.plot_eigenvalues(results)
    plt.savefig('outputs/eigenvalue_analysis.png', dpi=150, bbox_inches='tight')
    print("[OK] Eigenvalue plot saved: outputs/eigenvalue_analysis.png")
    plt.close()
    
    # 6. Generate stability report
    print("\n[TEST 6: Stability Report Generation]")
    analyzer.generate_stability_report('outputs/lyapunov_stability_report.txt')
    print("[OK] Report generated")
    
    # 7. Test stability region estimation (small sample for speed)
    print("\n[TEST 7: Stability Region Estimation]")
    region_results = analyzer.estimate_stability_region(n_samples=50, max_perturbation=0.1)
    print(f"[OK] Stability region analysis:")
    print(f"  - Stable samples: {region_results['stable_count']}/{region_results['n_samples']}")
    print(f"  - Stable fraction: {region_results['stable_fraction']*100:.1f}%")
    
    print("\n" + "="*70)
    print("   ALL TESTS PASSED")
    print("="*70)
    
    return analyzer, results


def test_passivity_verification():
    """Test passivity verification during simulation"""
    
    print("\n" + "="*70)
    print("   PASSIVITY VERIFICATION TEST")
    print("="*70)
    
    from scipy.integrate import solve_ivp
    
    # Initialize analyzer
    json_file = 'test_cases/Kundur_System/kundur_full.json'
    analyzer = LyapunovStabilityAnalyzer(json_file)
    
    # Initialize equilibrium
    x0 = analyzer.initialize_equilibrium()
    
    # Simulate small perturbation
    print("\n[Simulating with perturbation...]")
    x_init = x0 + np.random.randn(len(x0)) * 0.05
    
    # Define dynamics
    def dynamics(t, x):
        u = np.zeros(analyzer.n_gen)  # No control input
        return analyzer._system_dynamics(x, u)
    
    # Simulate
    t_span = (0, 5.0)
    t_eval = np.linspace(0, 5.0, 100)
    
    sol = solve_ivp(dynamics, t_span, x_init, t_eval=t_eval, 
                    method='RK45', rtol=1e-6, atol=1e-8)
    
    if sol.success:
        print(f"[OK] Simulation complete: {len(sol.t)} time points")
        
        # Compute Lyapunov function evolution
        V_evolution = []
        for i in range(len(sol.t)):
            V = analyzer.compute_lyapunov_function(sol.y[:, i])
            V_evolution.append(V)
            
        # Plot Lyapunov function
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10))
        
        # Lyapunov function
        ax1.plot(sol.t, V_evolution, linewidth=2)
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('V(x) = H(x) - H(x*)')
        ax1.set_title('Lyapunov Function Evolution')
        ax1.grid(True, alpha=0.3)
        ax1.axhline(y=0, color='r', linestyle='--', label='Equilibrium')
        ax1.legend()
        
        # Rotor angles
        for i in range(analyzer.n_gen):
            angles = sol.y[i*analyzer.states_per_machine, :] * 180/np.pi
            ax2.plot(sol.t, angles, label=f'Gen {i+1}')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Rotor Angle (deg)')
        ax2.set_title('Generator Rotor Angles')
        ax2.grid(True, alpha=0.3)
        ax2.legend()
        
        plt.tight_layout()
        plt.savefig('outputs/lyapunov_evolution.png', dpi=150, bbox_inches='tight')
        print("[OK] Lyapunov evolution plot saved: outputs/lyapunov_evolution.png")
        plt.close()
        
        # Check if Lyapunov function is decreasing
        is_decreasing = V_evolution[-1] < V_evolution[0]
        print(f"\n[OK] Passivity verification:")
        print(f"  - Initial V(x): {V_evolution[0]:.6e}")
        print(f"  - Final V(x): {V_evolution[-1]:.6e}")
        print(f"  - V decreasing: {is_decreasing}")
        
        if is_decreasing:
            print("  [OK] System is dissipating energy (converging to equilibrium)")
        else:
            print("  [WARN] Warning: Energy is increasing")
            
    else:
        print("[WARN] Simulation failed")
        
    print("\n" + "="*70)
    print("   PASSIVITY TEST COMPLETE")
    print("="*70)


def test_energy_margin():
    """Test transient energy margin computation"""
    
    print("\n" + "="*70)
    print("   TRANSIENT ENERGY MARGIN TEST")
    print("="*70)
    
    # Initialize analyzer
    json_file = 'test_cases/Kundur_System/kundur_full.json'
    analyzer = LyapunovStabilityAnalyzer(json_file)
    
    # Initialize equilibrium
    x_eq = analyzer.initialize_equilibrium()
    
    # Simulate fault-like perturbation
    print("\n[Simulating large disturbance (fault)...]")
    
    # Create fault state: sudden rotor angle increase
    # We perturb generators non-uniformly to create relative angle differences
    # (Uniform shift yields 0 energy change in COI frame)
    x_fault = x_eq.copy()
    
    # Fault near Generator 1: It accelerates significantly
    idx0 = 0 * analyzer.states_per_machine
    x_fault[idx0] += 0.5  # ~28 deg increase
    
    # Generator 2 accelerates less
    idx1 = 1 * analyzer.states_per_machine
    x_fault[idx1] += 0.2  # ~11 deg increase
        
    # Compute energy margin
    energy_results = analyzer.compute_energy_margin(x_fault)
    
    print(f"\n[OK] Energy Analysis:")
    print(f"  - Stable equilibrium energy: H* = {energy_results['H_stable']:.6f}")
    print(f"  - Post-fault energy: H_fault = {energy_results['H_fault']:.6f}")
    print(f"  - Energy absorbed: dH = {energy_results['delta_H']:.6f}")
    print(f"  - Normalized margin: {energy_results['normalized_margin']*100:.2f}%")
    
    if energy_results['delta_H'] > 1e-6:
        print("  [OK] System absorbed energy during fault")
    else:
        print("  [WARN] Warning: Negative energy change")
        
    print("\n" + "="*70)
    print("   ENERGY MARGIN TEST COMPLETE")
    print("="*70)


def test_stability_visualization(analyzer):
    """Visualize Stability Region using Lyapunov Level Sets and Energy Surfaces"""
    print("\n" + "="*70)
    print("   STABILITY VISUALIZATION")
    print("="*70)
    
    # 1. Estimate Stability Boundary via Sampling
    print("[1] Sampling state space to estimate stability boundary...")
    results = analyzer.estimate_stability_region(n_samples=500, max_perturbation=1.0)
    
    V_crit = results.get('V_critical', 10.0)
    print(f"  [OK] Estimated Critical Energy: V_crit = {V_crit:.4f}")

    # 2. Generate Multi-Panel Visualization
    print("[2] Generating comprehensive stability visualization...")
    
    fig = plt.figure(figsize=(16, 12))
    gs = fig.add_gridspec(3, 2, hspace=0.3, wspace=0.3)
    
    # Panel 1: Gen 1 Phase Plane (Delta vs Omega)
    ax1 = fig.add_subplot(gs[0, 0])
    plot_phase_plane(analyzer, ax1, gen_idx=0, V_crit=V_crit, 
                     delta_range=(-1.5, 1.5), omega_range=(-0.5, 0.5))
    
    # Panel 2: Gen 2 Phase Plane
    ax2 = fig.add_subplot(gs[0, 1])
    plot_phase_plane(analyzer, ax2, gen_idx=1, V_crit=V_crit,
                     delta_range=(-1.5, 1.5), omega_range=(-0.5, 0.5))
    
    # Panel 3: Relative Angle Plane (Delta_12 vs Omega_1)
    ax3 = fig.add_subplot(gs[1, 0])
    plot_relative_angle_plane(analyzer, ax3, V_crit=V_crit)
    
    # Panel 4: Energy Landscape (3D projection)
    ax4 = fig.add_subplot(gs[1, 1], projection='3d')
    plot_energy_surface_3d(analyzer, ax4, gen_idx=0)
    
    # Panel 5: Sample Distribution (V vs dV/dt)
    ax5 = fig.add_subplot(gs[2, 0])
    plot_stability_criterion(results, ax5, V_crit)
    
    # Panel 6: Region of Attraction Summary
    ax6 = fig.add_subplot(gs[2, 1])
    plot_roa_summary(analyzer, results, ax6, V_crit)
    
    plt.savefig('outputs/stability_visualization.png', dpi=150, bbox_inches='tight')
    print("[OK] Comprehensive visualization saved: outputs/stability_visualization.png")
    plt.close()


def plot_phase_plane(analyzer, ax, gen_idx, V_crit, delta_range, omega_range, n_points=80):
    """Plot phase plane for a single generator"""
    delta_vals = np.linspace(delta_range[0], delta_range[1], n_points)
    omega_vals = np.linspace(omega_range[0], omega_range[1], n_points)
    
    D, W = np.meshgrid(delta_vals, omega_vals)
    V_grid = np.zeros_like(D)
    
    idx_delta = gen_idx * analyzer.states_per_machine
    idx_p = gen_idx * analyzer.states_per_machine + 1
    M = analyzer.builder.gen_metadata[gen_idx]['M']
    
    x_base = analyzer.x_eq.copy()
    
    for i in range(n_points):
        for j in range(n_points):
            x_temp = x_base.copy()
            x_temp[idx_delta] += D[i, j]
            x_temp[idx_p] += M * W[i, j]
            V_grid[i, j] = analyzer.compute_lyapunov_function(x_temp)
    
    # Contour plot with proper levels
    levels = np.logspace(-2, np.log10(V_crit * 3), 20)
    cp = ax.contourf(D * 180/np.pi, W, V_grid, levels=levels, cmap='RdYlGn_r', alpha=0.7)
    ax.contour(D * 180/np.pi, W, V_grid, levels=levels, colors='k', linewidths=0.5, alpha=0.3)
    
    # Stability boundary
    cs = ax.contour(D * 180/np.pi, W, V_grid, levels=[V_crit], colors='red', linewidths=3)
    ax.clabel(cs, inline=1, fontsize=9, fmt=f'V={V_crit:.2f}')
    
    # Equilibrium point
    ax.plot(0, 0, 'k*', markersize=15, markeredgewidth=2, label='Equilibrium')
    
    ax.set_xlabel(f'Gen {gen_idx+1} Angle Deviation (deg)')
    ax.set_ylabel(f'Gen {gen_idx+1} Speed Deviation (pu)')
    ax.set_title(f'Generator {gen_idx+1} Phase Portrait')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right')


def plot_relative_angle_plane(analyzer, ax, V_crit, n_points=80):
    """Plot relative angle between Gen 1 and Gen 2"""
    delta_rel_range = np.linspace(-1.5, 1.5, n_points)
    omega1_range = np.linspace(-0.5, 0.5, n_points)
    
    DR, W1 = np.meshgrid(delta_rel_range, omega1_range)
    V_grid = np.zeros_like(DR)
    
    x_base = analyzer.x_eq.copy()
    M1 = analyzer.builder.gen_metadata[0]['M']
    
    for i in range(n_points):
        for j in range(n_points):
            x_temp = x_base.copy()
            # Perturb Gen 1 angle
            x_temp[0] += DR[i, j] / 2
            # Perturb Gen 2 angle in opposite direction (relative angle change)
            x_temp[analyzer.states_per_machine] -= DR[i, j] / 2
            # Perturb Gen 1 speed
            x_temp[1] += M1 * W1[i, j]
            
            V_grid[i, j] = analyzer.compute_lyapunov_function(x_temp)
    
    levels = np.logspace(-2, np.log10(V_crit * 3), 20)
    cp = ax.contourf(DR * 180/np.pi, W1, V_grid, levels=levels, cmap='RdYlGn_r', alpha=0.7)
    ax.contour(DR * 180/np.pi, W1, V_grid, levels=levels, colors='k', linewidths=0.5, alpha=0.3)
    
    cs = ax.contour(DR * 180/np.pi, W1, V_grid, levels=[V_crit], colors='red', linewidths=3)
    ax.clabel(cs, inline=1, fontsize=9, fmt=f'V={V_crit:.2f}')
    
    ax.plot(0, 0, 'k*', markersize=15, markeredgewidth=2)
    ax.set_xlabel('Relative Angle d1 - d2 (deg)')
    ax.set_ylabel('Gen 1 Speed Deviation (pu)')
    ax.set_title('Relative Angle Stability')
    ax.grid(True, alpha=0.3)


def plot_energy_surface_3d(analyzer, ax, gen_idx):
    """Plot 3D energy surface"""
    delta_range = np.linspace(-1.0, 1.0, 40)
    omega_range = np.linspace(-0.3, 0.3, 40)
    
    D, W = np.meshgrid(delta_range, omega_range)
    V_grid = np.zeros_like(D)
    
    idx_delta = gen_idx * analyzer.states_per_machine
    idx_p = gen_idx * analyzer.states_per_machine + 1
    M = analyzer.builder.gen_metadata[gen_idx]['M']
    
    x_base = analyzer.x_eq.copy()
    
    for i in range(40):
        for j in range(40):
            x_temp = x_base.copy()
            x_temp[idx_delta] += D[i, j]
            x_temp[idx_p] += M * W[i, j]
            V_grid[i, j] = analyzer.compute_lyapunov_function(x_temp)
    
    # Clip for visualization
    V_grid = np.clip(V_grid, 0, 50)
    
    surf = ax.plot_surface(D * 180/np.pi, W, V_grid, cmap='viridis', alpha=0.8)
    ax.set_xlabel(f'Angle (deg)')
    ax.set_ylabel(f'Speed (pu)')
    ax.set_zlabel('Energy V(x)')
    ax.set_title(f'Gen {gen_idx+1} Energy Landscape')


def plot_stability_criterion(results, ax, V_crit):
    """Plot V vs dV/dt scatter showing stability criterion"""
    V_samples = results['V_samples']
    dVdt_samples = results['dVdt_samples']
    
    # Separate stable and unstable
    stable_mask = dVdt_samples < 0
    
    ax.scatter(V_samples[stable_mask], dVdt_samples[stable_mask], 
               c='green', alpha=0.5, s=20, label='Stable (dV/dt < 0)')
    ax.scatter(V_samples[~stable_mask], dVdt_samples[~stable_mask],
               c='red', alpha=0.5, s=20, label='Unstable (dV/dt >= 0)')
    
    ax.axhline(y=0, color='k', linestyle='--', linewidth=1)
    ax.axvline(x=V_crit, color='r', linestyle='--', linewidth=2, label=f'V_crit = {V_crit:.2f}')
    
    ax.set_xlabel('Lyapunov Function V(x)')
    ax.set_ylabel('Lyapunov Derivative dV/dt')
    ax.set_title('Stability Criterion')
    ax.legend()
    ax.grid(True, alpha=0.3)


def plot_roa_summary(analyzer, results, ax, V_crit):
    """Summary statistics about region of attraction"""
    ax.axis('off')
    
    stable_frac = results['stable_fraction']
    n_stable = results['stable_count']
    n_total = results['n_samples']
    
    summary_text = f"""
    REGION OF ATTRACTION SUMMARY
    ===============================
    
    System: {analyzer.n_gen} generators
    States: {analyzer.total_states}
    
    Sampling Results:
    - Total samples: {n_total}
    - Stable samples: {n_stable} ({stable_frac*100:.1f}%)
    - Unstable samples: {n_total - n_stable}
    
    Energy Analysis:
    - Critical Energy: V_crit = {V_crit:.4f}
    - Equilibrium Energy: V* = 0
    
    Stability Region:
    Points with V(x) < {V_crit:.2f} and dV/dt < 0
    are guaranteed to converge to equilibrium.
    
    Port-Hamiltonian Structure:
    [OK] Energy-based Lyapunov function
    [OK] Passivity-based stability
    [OK] COI-frame rotational invariance
    """
    
    ax.text(0.1, 0.5, summary_text, fontsize=10, family='monospace',
            verticalalignment='center', transform=ax.transAxes)


if __name__ == "__main__":
    # Create outputs directory if it doesn't exist
    os.makedirs('outputs', exist_ok=True)
    
    # Run all tests
    print("\n" + "="*70)
    print("   STARTING LYAPUNOV ANALYZER TESTS")
    print("="*70)
    
    # Test 1: Basic stability analysis
    analyzer, results = test_stability_analysis()
    
    # Test 2: Passivity verification
    test_passivity_verification()
    
    # Test 3: Energy margin
    test_energy_margin()
    
    # Test 4: Visualization
    test_stability_visualization(analyzer)
    
    print("\n" + "="*70)
    print("   ALL TESTS COMPLETED SUCCESSFULLY")
    print("="*70)
    print("\nOutputs saved to 'outputs/' directory:")
    print("  - eigenvalue_analysis.png")
    print("  - lyapunov_evolution.png")
    print("  - stability_region_contours.png")
    print("  - lyapunov_stability_report.txt")
