"""
IEEE 39-Bus (New England) System - NO FAULT Simulation
Test that the system maintains steady-state equilibrium
"""
import sys
sys.path.insert(0, '.')

from utils.fault_sim_modular import ModularFaultSimulator
import matplotlib.pyplot as plt

print("="*70)
print(" IEEE 39-BUS (NEW ENGLAND) SYSTEM - NO FAULT EQUILIBRIUM TEST")
print("="*70)
print("\nSystem Components:")
print("  - 10 GENROU synchronous generators (Bus 30-39)")
print("  - 10 TGOV1 governors")
print("  - 10 IEEEX1 exciters")
print("  - 10 IEEEST power system stabilizers")
print("  - 10 BusFreq measurement devices")
print("  - 19 PQ loads, 2 shunt compensators")
print("  - 46 transmission lines/transformers")
print("  - Slack bus: Bus 39 (system equivalent, M=100)")
print("\nSimulation: NO FAULT - Check steady-state equilibrium")
print("-"*70)

# Initialize simulator with NO FAULT
sim = ModularFaultSimulator(
    system_json='test_cases/ieee39bus/ieee39_system.json',
    simulation_json='test_cases/ieee39bus/simulation_nofault.json'
)

# Initialize equilibrium
print("\n[1/3] Initializing equilibrium (power flow solving)...")
x0 = sim.initialize_equilibrium()

# Run short simulation to verify stability
print("\n[2/3] Running no-fault simulation (t=0 to 5s)...")
t_end = 5.0
n_points = 1000
sol = sim.simulate(x0, t_end=t_end, n_points=n_points)

# Plot results
print("\n[3/3] Plotting results...")
sim.plot_results(sol, filename='ieee39_nofault_equilibrium.png')

print("\n" + "="*70)
print("  SIMULATION COMPLETE")
print("="*70)
print(f"Output saved to: outputs/ieee39_nofault_equilibrium.png")
print("\nExpected result: All variables should remain constant (flat lines)")
print("  - Rotor angles: constant at initial values")
print("  - Speeds: constant at 1.0 p.u. (60 Hz)")
print("  - Field voltages: constant")
print("  - Mechanical power: constant")
print("="*70)
