"""Test modular fault simulator"""
import sys
sys.path.insert(0, '.')
from utils.fault_sim_modular import ModularFaultSimulator

# Example 1: Default fault
sim = ModularFaultSimulator('test_cases/Kundur_System/kundur_full.json')
x0 = sim.initialize_equilibrium()
sol = sim.simulate(x0, t_end=15.0)
sim.plot_results(sol)

# Example 2: Custom fault
# sim.set_fault(bus_idx=1, impedance=0.01j, start_time=2.0, duration=0.15)
# sol2 = sim.simulate(x0, t_end=15.0)
# sim.plot_results(sol2, filename='fault_simulation_custom.png')

# Example 3: No fault
# sim.disable_fault()
# sol3 = sim.simulate(x0, t_end=10.0)
