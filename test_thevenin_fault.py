"""Test modular fault simulator for Thevenin model"""
import sys
sys.path.insert(0, '.')
from utils.fault_sim_modular import ModularFaultSimulator

print("\n" + "="*70)
print("   FAULT SIMULATION TEST FOR THEVENIN MODEL")
print("="*70)

try:
    # Initialize simulator with system and simulation configs
    print("\n[Step 1] Initializing fault simulator...")
    sim = ModularFaultSimulator(
        system_json='test_cases/Thevenin_model/ieee_based_system.json',
        simulation_json='test_cases/Thevenin_model/simulation_fault.json'
    )
    
    # Initialize equilibrium
    print("\n[Step 2] Computing equilibrium point...")
    x0 = sim.initialize_equilibrium()
    
    # Run simulation with configured fault
    print("\n[Step 3] Running time-domain simulation...")
    print("  Fault configuration from simulation JSON:")
    print(f"    - Fault bus: {sim.fault_bus}")
    print(f"    - Fault impedance: {sim.fault_impedance}")
    print(f"    - Fault start time: {sim.fault_start}s")
    print(f"    - Fault duration: {sim.fault_duration}s")
    
    sol = sim.simulate(x0, t_end=sim.t_end)
    
    # Plot results
    print("\n[Step 4] Generating plots...")
    sim.plot_results(sol, filename='thevenin_fault_simulation.png')
    
    print("\n" + "="*70)
    print("   SUCCESS: Fault simulation completed!")
    print("   Output saved to: outputs/thevenin_fault_simulation.png")
    print("="*70)
    
except Exception as e:
    import traceback
    print("\n" + "="*70)
    print("   ERROR: Fault simulation failed")
    print("="*70)
    print(f"\nError message: {e}\n")
    traceback.print_exc()
    sys.exit(1)
