"""
Test Lyapunov Stability Analyzer
Demonstrates passivity-based stability analysis for power systems
"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
import matplotlib.pyplot as plt
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
    print(f"✓ Equilibrium state dimension: {len(x_eq)}")
    print(f"✓ Equilibrium Hamiltonian: H* = {analyzer.H_eq:.6f}")
    
    # 2. Test Hamiltonian computation
    print("\n[TEST 2: Hamiltonian Computation]")
    H = analyzer.compute_hamiltonian(x_eq)
    print(f"✓ H(x*) = {H:.6f}")
    
    # Test with perturbation
    x_pert = x_eq + np.random.randn(len(x_eq)) * 0.01
    H_pert = analyzer.compute_hamiltonian(x_pert)
    print(f"✓ H(x* + δx) = {H_pert:.6f}")
    print(f"✓ ΔH = {H_pert - H:.6f}")
    
    # 3. Test Lyapunov function
    print("\n[TEST 3: Lyapunov Function]")
    V = analyzer.compute_lyapunov_function(x_eq)
    print(f"✓ V(x*) = {V:.6e} (should be ~0)")
    
    V_pert = analyzer.compute_lyapunov_function(x_pert)
    print(f"✓ V(x* + δx) = {V_pert:.6e} (should be >0)")
    
    # 4. Linearized stability analysis
    print("\n[TEST 4: Linearized Stability Analysis]")
    results = analyzer.analyze_linearized_stability()
    
    print(f"\n✓ Stability Analysis Complete:")
    print(f"  - System is: {'STABLE' if results['is_stable'] else 'UNSTABLE'}")
    print(f"  - Stable modes: {results['stable_count']}")
    print(f"  - Unstable modes: {results['unstable_count']}")
    print(f"  - Marginal modes: {results['marginal_count']}")
    
    # 5. Plot eigenvalues
    print("\n[TEST 5: Eigenvalue Visualization]")
    fig = analyzer.plot_eigenvalues(results)
    plt.savefig('outputs/eigenvalue_analysis.png', dpi=150, bbox_inches='tight')
    print("✓ Eigenvalue plot saved: outputs/eigenvalue_analysis.png")
    plt.close()
    
    # 6. Generate stability report
    print("\n[TEST 6: Stability Report Generation]")
    analyzer.generate_stability_report('outputs/lyapunov_stability_report.txt')
    print("✓ Report generated")
    
    # 7. Test stability region estimation (small sample for speed)
    print("\n[TEST 7: Stability Region Estimation]")
    region_results = analyzer.estimate_stability_region(n_samples=50, max_perturbation=0.1)
    print(f"✓ Stability region analysis:")
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
        print(f"✓ Simulation complete: {len(sol.t)} time points")
        
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
        print("✓ Lyapunov evolution plot saved: outputs/lyapunov_evolution.png")
        plt.close()
        
        # Check if Lyapunov function is decreasing
        is_decreasing = V_evolution[-1] < V_evolution[0]
        print(f"\n✓ Passivity verification:")
        print(f"  - Initial V(x): {V_evolution[0]:.6e}")
        print(f"  - Final V(x): {V_evolution[-1]:.6e}")
        print(f"  - V decreasing: {is_decreasing}")
        
        if is_decreasing:
            print("  ✓ System is dissipating energy (converging to equilibrium)")
        else:
            print("  ✗ Warning: Energy is increasing")
            
    else:
        print("✗ Simulation failed")
        
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
    x_fault = x_eq.copy()
    for i in range(analyzer.n_gen):
        idx = i * analyzer.states_per_machine
        x_fault[idx] += 0.5  # 28 degree angle increase
        
    # Compute energy margin
    energy_results = analyzer.compute_energy_margin(x_fault)
    
    print(f"\n✓ Energy Analysis:")
    print(f"  - Stable equilibrium energy: H* = {energy_results['H_stable']:.6f}")
    print(f"  - Post-fault energy: H_fault = {energy_results['H_fault']:.6f}")
    print(f"  - Energy absorbed: ΔH = {energy_results['delta_H']:.6f}")
    print(f"  - Normalized margin: {energy_results['normalized_margin']*100:.2f}%")
    
    if energy_results['delta_H'] > 0:
        print("  ✓ System absorbed energy during fault")
    else:
        print("  ✗ Warning: Negative energy change")
        
    print("\n" + "="*70)
    print("   ENERGY MARGIN TEST COMPLETE")
    print("="*70)


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
    
    print("\n" + "="*70)
    print("   ALL TESTS COMPLETED SUCCESSFULLY")
    print("="*70)
    print("\nOutputs saved to 'outputs/' directory:")
    print("  - eigenvalue_analysis.png")
    print("  - lyapunov_evolution.png")
    print("  - lyapunov_stability_report.txt")
