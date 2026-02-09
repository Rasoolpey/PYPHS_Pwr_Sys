"""IEEE 14-Bus Fault Simulation (Modular)

Runs the 14-bus system with a single-line-to-ground fault at Bus 9.
Uses power-flow-based initialization and the modular fault simulator.
"""
import sys
sys.path.insert(0, '.')

from utils.fault_sim_modular import ModularFaultSimulator

SYSTEM_JSON = 'test_cases/ieee14bus/ieee14_system.json'
SIM_JSON = 'test_cases/ieee14bus/simulation_fault_full.json'


def main():
    print("=" * 70)
    print(" IEEE 14-BUS SYSTEM - FAULT SIMULATION")
    print("=" * 70)
    print("\nConfig:")
    print(f"  System: {SYSTEM_JSON}")
    print(f"  Simulation: {SIM_JSON}")
    print("  Fault: Bus 9, t=1.0-1.1s, Z=j0.001 pu")
    print("  Solver: Radau (rtol=1e-6, atol=1e-8)")
    print("  Init: Power flow enabled\n")

    # Build simulator with explicit simulation JSON
    sim = ModularFaultSimulator(system_json=SYSTEM_JSON, simulation_json=SIM_JSON)

    # Initialize using power flow
    print("[1/3] Initializing equilibrium (power flow)...")
    x0 = sim.initialize_equilibrium()

    # Run faulted simulation
    print("\n[2/3] Running fault simulation...")
    sol_fault = sim.simulate(x0, t_end=sim.t_end, n_points=sim.n_points)

    # Plot faulted results
    print("\n[3/3] Plotting results...")
    sim.plot_results(sol_fault, filename='ieee14_fault_simulation.png')

    print("\nOutputs:")
    print("  Faulted: outputs/ieee14_fault_simulation.png")

    # Optional: no-fault baseline for comparison
    print("\nRunning no-fault baseline for comparison...")
    sim.disable_fault()
    sol_nf = sim.simulate(x0, t_end=sim.t_end, n_points=sim.n_points)
    sim.plot_results(sol_nf, filename='ieee14_nofault_simulation.png')
    print("  No-fault: outputs/ieee14_nofault_simulation.png")


if __name__ == '__main__':
    main()
