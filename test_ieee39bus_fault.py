"""IEEE 39-Bus Fault Simulation (Modular)

Runs the New England 39-bus system with a three-phase fault at Bus 2.
Uses power-flow-based initialization and the modular fault simulator.
Also produces a no-fault baseline for comparison.
"""
import sys
sys.path.insert(0, '.')

from utils.fault_sim_modular import ModularFaultSimulator

SYSTEM_JSON = 'test_cases/ieee39bus/ieee39_system.json'
SIM_JSON_FAULT = 'test_cases/ieee39bus/simulation_fault.json'
SIM_JSON_NOFAULT = 'test_cases/ieee39bus/simulation_nofault.json'


def run_case(sim_json, label, plot_name):
    sim = ModularFaultSimulator(system_json=SYSTEM_JSON, simulation_json=sim_json)
    print(f"[Init] {label}: running power-flow-based initialization...")
    x0 = sim.initialize_equilibrium()
    print(f"[Sim ] {label}: running t=0..{sim.t_end}s, fault={'on' if sim.fault_enabled else 'off'}")
    sol = sim.simulate(x0, t_end=sim.t_end, n_points=sim.n_points)
    sim.plot_results(sol, filename=plot_name)
    return sol


def main():
    print("=" * 70)
    print(" IEEE 39-BUS SYSTEM - FAULT SIMULATION")
    print("=" * 70)
    print("  Fault: Bus 2, t=1.0-1.1s, Z=j0.005 pu")
    print("  Solver: Radau (rtol=1e-6, atol=1e-8)")
    print("  Init: Power flow enabled\n")

    # Faulted run
    sol_fault = run_case(SIM_JSON_FAULT, "Faulted", "ieee39_fault_simulation.png")
    print("  Faulted plot: outputs/ieee39_fault_simulation.png")

    # No-fault baseline
    sol_nf = run_case(SIM_JSON_NOFAULT, "No-fault", "ieee39_nofault_simulation.png")
    print("  No-fault plot: outputs/ieee39_nofault_simulation.png")

    # Quick summary
    print("\nSummary:")
    print(f"  Faulted steps: {len(sol_fault.t)}  final t={sol_fault.t[-1]:.2f}s")
    print(f"  No-fault steps: {len(sol_nf.t)}  final t={sol_nf.t[-1]:.2f}s")


if __name__ == '__main__':
    main()
