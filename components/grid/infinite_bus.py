"""Infinite Bus / Grid Model - Maintains constant voltage"""
import sys
sys.path.insert(0, '/home/claude')
from utils.pyphs_core import DynamicsCore
import numpy as np


def infinite_bus_dynamics(x, ports, meta):
    """
    Dynamics for infinite bus (voltage source).
    
    The infinite bus maintains constant voltage and angle.
    In a true voltage source, there are no dynamics - V and theta are fixed.
    
    However, for numerical integration, we include a very fast "stiff" dynamics
    that drives the bus back to its reference values.
    
    Args:
        x: numpy array of 2 states [V_magnitude, theta_angle]
        ports: dict with keys {'Id', 'Iq', 'P_demand', 'Q_demand'}
        meta: dict of parameters
    
    Returns:
        x_dot: numpy array of 2 state derivatives
    """
    V_mag, theta = x
    
    # Extract reference values
    V_ref = meta.get('V_ref', 1.0)
    theta_ref = meta.get('theta_ref', 0.0)
    
    # Very fast dynamics to maintain voltage (stiff system)
    T_voltage = meta.get('T_voltage', 0.001)  # Very fast response
    T_angle = meta.get('T_angle', 0.001)
    
    x_dot = np.zeros(2)
    
    # Drive voltage back to reference
    x_dot[0] = (V_ref - V_mag) / T_voltage
    x_dot[1] = (theta_ref - theta) / T_angle
    
    return x_dot


def infinite_bus_output(x, ports, meta):
    """
    Compute voltage phasor output.
    
    Args:
        x: numpy array of states
        ports: dict with current info
        meta: dict of parameters
    
    Returns:
        V_phasor: complex voltage phasor
    """
    V_mag, theta = x
    V_phasor = V_mag * np.exp(1j * theta)
    return V_phasor


def build_infinite_bus_core(grid_data):
    """Build Infinite Bus as DynamicsCore
    
    Args:
        grid_data: dict with grid parameters from Slack data
    
    Returns:
        core: DynamicsCore object with dynamics method
        metadata: dict with additional info
    """
    core = DynamicsCore(label=f'InfiniteBus_{grid_data["idx"]}', dynamics_fn=infinite_bus_dynamics)
    
    # Extract parameters
    V_ref = grid_data.get('v0', 1.0)
    theta_ref = grid_data.get('a0', 0.0)
    bus_idx = grid_data['bus']
    
    # States: [V_magnitude, theta_angle]
    V_mag, theta = core.symbols(['V_grid', 'theta_grid'])
    
    # Parameters
    V_ref_sym, theta_ref_sym = core.symbols(['V_ref_grid', 'theta_ref_grid'])
    T_voltage_sym = core.symbols('T_voltage_grid')
    
    # Hamiltonian (minimal energy - grid is stiff)
    H_grid = (T_voltage_sym/2) * ((V_mag - V_ref_sym)**2 + (theta - theta_ref_sym)**2)
    
    core.add_storages([V_mag, theta], H_grid)
    
    # Ports: Output [V_phasor], Input [I_phasor]
    I_in = core.symbols('I_grid_in')
    V_out = core.symbols('V_grid_out')
    
    core.add_ports([I_in], [V_out])
    
    # Parameter substitutions
    core.subs.update({
        V_ref_sym: V_ref,
        theta_ref_sym: theta_ref,
        T_voltage_sym: 0.001
    })
    
    # Metadata
    metadata = {
        'idx': grid_data['idx'],
        'bus': bus_idx,
        'V_ref': V_ref,
        'theta_ref': theta_ref,
        'T_voltage': 0.001,
        'T_angle': 0.001,
        'Sn': grid_data.get('Sn', 100.0),
        'Vn': grid_data.get('Vn', 345.0)
    }
    
    # Set metadata on core
    core.set_metadata(metadata)
    
    # Set component interface attributes
    core.n_states = 2
    core.output_fn = infinite_bus_output
    core.component_type = "grid"
    core.model_name = "InfiniteBus"
    
    return core, metadata
