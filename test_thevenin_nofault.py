"""Test Thevenin model without fault - verify base stability"""
import sys
import numpy as np
sys.path.insert(0, '.')
from utils.fault_sim_modular import ModularFaultSimulator

print("\n" + "="*70)
print("   STABILITY TEST FOR THEVENIN MODEL (NO FAULT)")
print("="*70)

try:
    # Initialize simulator with system and no-fault simulation config
    print("\n[Step 1] Initializing simulator...")
    sim = ModularFaultSimulator(
        system_json='test_cases/Thevenin_model/ieee_based_system.json',
        simulation_json='test_cases/Thevenin_model/simulation_nofault.json'
    )
    
    # Initialize equilibrium
    print("\n[Step 2] Computing equilibrium point...")
    x0 = sim.initialize_equilibrium()
    
    # Run simulation without fault
    print("\n[Step 3] Running time-domain simulation (no fault)...")
    sol = sim.simulate(x0, t_end=sim.t_end)
    
    # Plot results
    print("\n[Step 4] Generating plots...")
    sim.plot_results(sol, filename='thevenin_no_fault.png')
    
    # Check stability
    angles = sol.y[0::13, :]  # Extract rotor angles
    final_angle = angles[0, -1]
    angle_drift = abs(final_angle - angles[0, 0])
    
    print("\n" + "-"*70)
    print(f"Initial angle: {np.degrees(angles[0, 0]):.2f} deg")
    print(f"Final angle:   {np.degrees(final_angle):.2f} deg")
    print(f"Angle drift:   {np.degrees(angle_drift):.2f} deg")
    
    if angle_drift < np.radians(10):  # Less than 10 degrees drift
        print("Status: STABLE")
    else:
        print("Status: UNSTABLE or DRIFTING")
    print("-"*70)
    
    print("\n" + "="*70)
    print("   SUCCESS: No-fault simulation completed!")
    print("   Output saved to: outputs/thevenin_no_fault.png")
    print("="*70)
    
except Exception as e:
    import traceback
    print("\n" + "="*70)
    print("   ERROR: Simulation failed")
    print("="*70)
    print(f"\nError message: {e}\n")
    traceback.print_exc()
    sys.exit(1)
