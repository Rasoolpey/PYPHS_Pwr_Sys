"""
IEEE 14-Bus System - NO FAULT Simulation
Test that the system maintains steady-state equilibrium
"""
import sys
sys.path.insert(0, 'h:/Thesis/Gitcodes/Kundur_PYPHS_Dynamics')

from utils.fault_sim_modular import ModularFaultSimulator
import matplotlib.pyplot as plt

print("="*70)
print(" IEEE 14-BUS SYSTEM - NO FAULT EQUILIBRIUM TEST")
print("="*70)
print("\nSystem Components:")
print("  - 5 GENROU synchronous generators")
print("  - 3 TGOV1 governors (Gen 1, 4, 5)")
print("  - 2 IEEEG1 governors (Gen 2, 3)")
print("  - 1 EXST1 exciter (Gen 2)")
print("  - 4 ESST3A exciters (Gen 1, 3, 4, 5)")
print("  - 1 IEEEST PSS on ESST3A_3")
print("  - 2 ST2CUT PSS on ESST3A_2, EXST1_1")
print("  - 3 BusFreq measurement devices")
print("\nSimulation: NO FAULT - Check steady-state equilibrium")
print("-"*70)

# Initialize simulator with NO FAULT
sim = ModularFaultSimulator(
    system_json='test_cases/ieee14bus/ieee14_system.json',
    simulation_json=None  # Will use defaults, then disable fault
)

# Disable fault for this test
sim.fault_enabled = False

# Initialize equilibrium
print("\n[1/2] Initializing equilibrium (power flow solving)...")
x0 = sim.initialize_equilibrium()

# Run short simulation to verify stability
print("\n[2/2] Running no-fault simulation (t=0 to 5s)...")
t_end = 5.0
n_points = 1000
sol = sim.simulate(x0, t_end=t_end, n_points=n_points)

# Plot results
print("\n[3/3] Plotting results...")
sim.plot_results(sol, filename='ieee14_nofault_equilibrium.png')

print("\n" + "="*70)
print("  SIMULATION COMPLETE")
print("="*70)
print(f"Output saved to: outputs/ieee14_nofault_equilibrium.png")
print("\nExpected result: All variables should remain constant (flat lines)")
print("  - Rotor angles: constant at initial values")
print("  - Speeds: constant at 1.0 p.u. (60 Hz)")
print("  - Field voltages: constant")
print("  - Mechanical power: constant")
print("="*70)
