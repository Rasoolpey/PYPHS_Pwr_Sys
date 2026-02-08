"""
Demonstrate power flow initialization module.

This shows how to use the general power flow solver to find
accurate initial conditions before RMS simulation.
"""
import sys
sys.path.insert(0, '.')
from utils.fault_sim_modular import ModularFaultSimulator

print("\n" + "="*70)
print("   POWER FLOW INITIALIZATION DEMONSTRATION")
print("="*70)

try:
    # Initialize simulator
    print("\n[Step 1] Initializing simulator...")
    sim = ModularFaultSimulator(
        system_json='test_cases/Thevenin_model/ieee_based_system.json',
        simulation_json='test_cases/Thevenin_model/simulation_fault.json'
    )
    
    print("\n[Step 2] Solving AC power flow for accurate initial conditions...")
    converged = sim.solve_power_flow(verbose=True)
    
    if not converged:
        print("\nWARNING: Power flow did not converge!")
        print("Continuing with initial guess from JSON...")
    
    # Initialize equilibrium (will use power flow results if available)
    print("\n[Step 3] Computing dynamic equilibrium from power flow...")
    x0 = sim.initialize_equilibrium(run_power_flow=False)  # Already solved above
    
    # Run simulation
    print("\n[Step 4] Running time-domain simulation...")
    sol = sim.simulate(x0, t_end=sim.t_end)
    
    # Plot results
    print("\n[Step 5] Generating plots...")
    sim.plot_results(sol, filename='thevenin_with_powerflow.png')
    
    print("\n" + "="*70)
    print("   SUCCESS: Power flow initialization completed!")
    print("   Output saved to: outputs/thevenin_with_powerflow.png")
    print("="*70)
    print("\nKey Benefits of Power Flow Initialization:")
    print("  + Accurate voltage magnitudes and angles")
    print("  + Consistent power injections and flows")
    print("  + Better equilibrium for dynamic simulation")
    print("  + Reduced initialization transients")
    print("="*70)
    
except Exception as e:
    import traceback
    print("\n" + "="*70)
    print("   ERROR: Demonstration failed")
    print("="*70)
    print(f"\nError message: {e}\n")
    traceback.print_exc()
    sys.exit(1)
