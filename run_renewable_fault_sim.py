"""
Full fault simulation with renewable energy
IEEE 14-bus system with Type-3 Wind Turbine at bus 8
"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from utils.fault_sim_modular import ModularFaultSimulator

def main():
    # System and simulation configs
    system_json = "test_cases/ieee14bus/renewable_resources_adoption/ieee14_wt3_system.json"
    sim_json = "test_cases/ieee14bus/renewable_resources_adoption/simulation_fault.json"
    
    print("\n" + "="*80)
    print("  IEEE 14-BUS FAULT SIMULATION WITH TYPE-3 WIND TURBINE")
    print("="*80)
    
    # Create simulator
    sim = ModularFaultSimulator(system_json=system_json, simulation_json=sim_json)
    
    # Initialize with power flow
    x0 = sim.initialize_equilibrium(run_power_flow=True)
    
    # Run simulation (config specifies 15s, bus 9 fault at 1.0-1.1s)
    print(f"\nRunning {sim.t_end}s simulation...")
    print(f"Fault: Bus {sim.fault_bus}, {sim.fault_start}-{sim.fault_start+sim.fault_duration}s, Z={sim.fault_impedance}")
    
    sol = sim.simulate(x0, t_end=sim.t_end, n_points=sim.n_points)
    
    if sol.success:
        print("\n" + "="*80)
        print("  SIMULATION COMPLETED SUCCESSFULLY")
        print("="*80)
        
        # Plot results
        sim.plot_results(sol, filename='renewable_fault_simulation.png')
        
    else:
        print("\n" + "="*80)
        print("  SIMULATION FAILED")
        print("="*80)
        print(f"Message: {sol.message}")
        return 1
    
    return 0

if __name__ == "__main__":
    sys.exit(main())
