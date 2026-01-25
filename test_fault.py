"""Test fault simulation with PyPHS components"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from utils.fault_simulator import FaultSimulator

json_path = os.path.join(os.path.dirname(__file__), 'test_cases', 'Kundur_System', 'kundur_full.json')
sim = FaultSimulator(json_path)
x0 = sim.initialize_equilibrium()
sol = sim.simulate(x0, t_end=15.0)
if sol and sol.success:
    sim.plot_results(sol)
