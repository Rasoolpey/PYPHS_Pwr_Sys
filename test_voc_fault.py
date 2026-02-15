"""
IEEE 14-Bus + VOC Inverter - WITH FAULT Simulation
Tests transient stability with Virtual Oscillator Control grid-forming inverter at bus 8
Fault: 3-phase short circuit at Bus 9, duration 50ms
"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
from utils.fault_sim_modular import ModularFaultSimulator

def main():
    print("\n" + "="*80)
    print("  IEEE 14-BUS + VOC INVERTER - FAULT SIMULATION")
    print("  Virtual Oscillator Control with P-f Droop (mp=0.05, H_virtual~3.2s)")
    print("  Fault: Bus 9, 50ms duration, Z=0.01j ohm")
    print("="*80)

    system_json = "test_cases/ieee14bus/renewable_resources_adoption/ieee14_voc_system.json"
    sim_json = "test_cases/ieee14bus/renewable_resources_adoption/simulation_fault.json"

    # Create simulator
    print("\n[1/4] Creating simulator...")
    sim = ModularFaultSimulator(system_json=system_json, simulation_json=sim_json)
    
    print(f"\n  System Configuration:")
    print(f"    - Synchronous Generators: {sim.n_gen} units")
    print(f"    - Total Inertia (Sync Only): {sim.M_total:.1f} MWs/MVA")
    print(f"    - VOC Inverters: {sim.n_voc} unit(s)")
    if sim.n_voc > 0:
        total_voc_inertia = sum(meta.get('H_virtual', 0) * meta.get('Sn', 100) / 100 
                                for meta in sim.builder.ren_voc_metadata)
        print(f"    - VOC Virtual Inertia: {total_voc_inertia:.1f} MWs/MVA")
        print(f"    - Effective Total Inertia: {sim.M_total + total_voc_inertia:.1f} MWs/MVA")
        print(f"\n  Expected Improvement vs GFL:")
        print(f"    - GFL (WT3):  M_eff = 41.0 MWs/MVA, CCT ~ 50ms")
        print(f"    - VOC:        M_eff = {sim.M_total + total_voc_inertia:.1f} MWs/MVA, CCT ~ 75-80ms (estimated)")

    # Initialize with power flow
    print("\n[2/4] Initializing with power flow...")
    x0 = sim.initialize_equilibrium(run_power_flow=True)
    print(f"  Initialized with {len(x0)} states")

    # Enable fault (should already be enabled from JSON)
    print(f"\n  Fault Configuration:")
    print(f"    - Location: Bus {sim.fault_bus}")
    print(f"    - Start time: {sim.fault_start:.2f}s")
    print(f"    - Duration: {sim.fault_duration*1000:.0f}ms")
    print(f"    - Impedance: {sim.fault_impedance}")

    # Run simulation
    t_end = 15.0
    n_points = 3000
    print(f"\n[3/4] Running fault simulation (t=0 to {t_end}s)...")
    sol = sim.simulate(x0, t_end=t_end, n_points=n_points)

    # Save results
    print("\n[4/4] Saving results...")
    sim.plot_results(sol, filename='voc_fault_simulation.png')
    print(f"  Plot saved to: outputs/voc_fault_simulation.png")

    # Save raw data
    np.savez('outputs/voc_fault_data.npz', t=sol.t, y=sol.y)
    print(f"  Raw data saved to: outputs/voc_fault_data.npz")

    # Summary
    print("\n" + "="*80)
    print("  SIMULATION COMPLETE")
    print("="*80)
    print(f"  Time points: {len(sol.t)}")
    print(f"  Final time: {sol.t[-1]:.2f}s")
    if sol.t[-1] >= t_end - 0.1:
        print(f"  Status: [STABLE] - System maintained synchronism")
        print(f"  VOC virtual inertia helped system ride through the fault!")
    else:
        print(f"  Status: [UNSTABLE] - System lost synchronism at t={sol.t[-1]:.2f}s")
        print(f"  Consider: Increase droop gain (mp) for more virtual inertia")
    print("="*80)

if __name__ == "__main__":
    main()
