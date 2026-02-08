"""Test IEEE 14-Bus System with Full Dynamics

This script tests the complete IEEE 14-bus benchmark system with:
- 5 GENROU generators
- Mixed governor controls (3x TGOV1, 2x IEEEG1)
- Mixed exciter controls (1x EXST1, 4x ESST3A)
- 3 Power System Stabilizers (1x IEEEST, 2x ST2CUT)
- 3 BusFreq measurement devices
- Fault at Bus 9 (t=1.0-1.1s)
"""
import sys
sys.path.insert(0, '.')
from utils.fault_sim_modular import ModularFaultSimulator

# IEEE 14-Bus System with complete dynamics
print("=" * 70)
print(" IEEE 14-BUS SYSTEM - COMPLETE DYNAMIC MODEL SIMULATION")
print("=" * 70)
print("\nSystem Components:")
print("  - 5 GENROU synchronous generators")
print("  - 3 TGOV1 governors (Gen 1, 4, 5)")
print("  - 2 IEEEG1 governors (Gen 2, 3)")
print("  - 1 EXST1 exciter (Gen 2)")
print("  - 4 ESST3A exciters (Gen 1, 3, 4, 5)")
print("  - 1 IEEEST PSS on ESST3A_3")
print("  - 2 ST2CUT PSS on ESST3A_2, EXST1_1")
print("  - 3 BusFreq measurement devices")
print("\nSimulation: Fault at Bus 9, t=1.0s, duration=0.1s")
print("-" * 70)

# Initialize simulator
sim = ModularFaultSimulator('test_cases/ieee14bus/ieee14_system.json')

# Initialize equilibrium with power flow
print("\n[1/3] Initializing equilibrium (power flow solving)...")
x0 = sim.initialize_equilibrium()

# Run fault simulation
print("\n[2/3] Running simulation (t=0 to 15s)...")
sol = sim.simulate(x0, t_end=15.0)

# Plot results
print("\n[3/3] Generating plots...")
sim.plot_results(sol, filename='ieee14_fault_simulation.png')

print("\n" + "=" * 70)
print(" SIMULATION COMPLETE")
print("=" * 70)
print(f"\nResults saved to: outputs/ieee14_fault_simulation.png")
print(f"Total simulation time: {sol.t[-1]:.2f} seconds")
print(f"Integration steps: {len(sol.t)}")
print(f"Final equilibrium quality: Max |dx/dt| = {max(abs(sol.y[:, -1])):.2e}")

# Optional: Run no-fault simulation for comparison
print("\n" + "-" * 70)
print("Running no-fault simulation for comparison...")
sim.disable_fault()
sol_nofault = sim.simulate(x0, t_end=15.0)
sim.plot_results(sol_nofault, filename='ieee14_nofault_simulation.png')
print("No-fault results saved to: outputs/ieee14_nofault_simulation.png")
