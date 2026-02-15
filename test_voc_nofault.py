"""
IEEE 14-Bus + VOC Inverter - NO FAULT Simulation
Tests steady-state stability with Virtual Oscillator Control grid-forming inverter at bus 8
"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
from utils.fault_sim_modular import ModularFaultSimulator

def main():
    print("\n" + "="*80)
    print("  IEEE 14-BUS + VOC INVERTER - NO FAULT EQUILIBRIUM TEST")
    print("  Virtual Oscillator Control with P-f Droop (mp=0.05, H_virtual~3.2s)")
    print("="*80)

    system_json = "test_cases/ieee14bus/renewable_resources_adoption/ieee14_voc_system.json"
    sim_json = "test_cases/ieee14bus/renewable_resources_adoption/simulation_fault.json"

    # Create simulator
    print("\n[1/4] Creating simulator...")
    sim = ModularFaultSimulator(system_json=system_json, simulation_json=sim_json)
    
    print(f"\n  System Configuration:")
    print(f"    - Synchronous Generators: {sim.n_gen} units")
    print(f"    - VOC Inverters: {sim.n_voc} unit(s)")
    print(f"    - Total States: {sim.total_states}")
    if sim.n_voc > 0:
        for v in range(sim.n_voc):
            voc_meta = sim.builder.ren_voc_metadata[v]
            print(f"\n  VOC Inverter {v+1} ({voc_meta['idx']}) at Bus {voc_meta['bus']}:")
            print(f"    - Virtual Inertia: H = {voc_meta.get('H_virtual', 0):.2f} seconds")
            print(f"    - Droop: {voc_meta.get('droop_percentage', 0):.1f}% (mp={voc_meta['mp']})")
            print(f"    - Power Setpoint: P={voc_meta['Pref']:.3f} pu, Q={voc_meta['Qref']:.3f} pu")

    # Initialize with power flow
    print("\n[2/4] Initializing with power flow...")
    x0 = sim.initialize_equilibrium(run_power_flow=True)
    print(f"  Initialized with {len(x0)} states")

    # Disable fault for equilibrium test
    sim.disable_fault()

    # Run simulation
    t_end = 15.0  # Shorter than WT3 test for initial validation
    n_points = 3000
    print(f"\n[3/4] Running no-fault simulation (t=0 to {t_end}s)...")
    sol = sim.simulate(x0, t_end=t_end, n_points=n_points)

    # Save results
    print("\n[4/4] Saving results...")
    sim.plot_results(sol, filename='voc_nofault_simulation.png')
    print(f"  Plot saved to: outputs/voc_nofault_simulation.png")

    # Save raw data
    np.savez('outputs/voc_nofault_data.npz', t=sol.t, y=sol.y)
    print(f"  Raw data saved to: outputs/voc_nofault_data.npz")

    # Summary
    print("\n" + "="*80)
    print("  SIMULATION COMPLETE")
    print("="*80)
    print(f"  Time points: {len(sol.t)}")
    print(f"  Final time: {sol.t[-1]:.2f}s")
    print(f"  Expected: All variables should remain constant (flat lines)")
    print(f"  VOC provides virtual inertia - system inertia effectively increased")
    print("="*80)

if __name__ == "__main__":
    main()
