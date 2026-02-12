"""
IEEE 14-Bus + WT3 Renewable - FAULT Simulation
Tests transient stability with wind turbine at bus 8
Fault at bus 9, t=1.0-1.1s
"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
from utils.fault_sim_modular import ModularFaultSimulator

def main():
    print("\n" + "="*80)
    print("  IEEE 14-BUS + WT3 RENEWABLE - FAULT SIMULATION")
    print("="*80)

    system_json = "test_cases/ieee14bus/renewable_resources_adoption/ieee14_wt3_system.json"
    sim_json = "test_cases/ieee14bus/renewable_resources_adoption/simulation_fault.json"

    # Create simulator
    print("\n[1/4] Creating simulator...")
    sim = ModularFaultSimulator(system_json=system_json, simulation_json=sim_json)

    # Initialize with power flow
    print("\n[2/4] Initializing with power flow...")
    x0 = sim.initialize_equilibrium(run_power_flow=True)
    print(f"  Initialized with {len(x0)} states")

    # Fault is already configured from simulation JSON
    print(f"  Fault: bus={sim.fault_bus}, Z={sim.fault_impedance}, "
          f"t={sim.fault_start}-{sim.fault_start + sim.fault_duration}s")

    # Run simulation
    t_end = 15.0
    n_points = 3000
    print(f"\n[3/4] Running fault simulation (t=0 to {t_end}s)...")
    sol = sim.simulate(x0, t_end=t_end, n_points=n_points)

    # Save results
    print("\n[4/4] Saving results...")
    sim.plot_results(sol, filename='renewable_fault_simulation.png')
    print(f"  Plot saved to: outputs/renewable_fault_simulation.png")

    # Save raw data
    np.savez('outputs/renewable_fault_data.npz', t=sol.t, y=sol.y)
    print(f"  Raw data saved to: outputs/renewable_fault_data.npz")

    # Summary
    print("\n" + "="*80)
    print("  SIMULATION COMPLETE")
    print("="*80)
    print(f"  Time points: {len(sol.t)}")
    print(f"  Final time: {sol.t[-1]:.2f}s")
    print("="*80)

if __name__ == "__main__":
    main()
