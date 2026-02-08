"""TGOV1 Governor Model"""
import sys
sys.path.insert(0, '/home/claude')
from utils.pyphs_core import DynamicsCore
import numpy as np


def tgov1_dynamics(x, ports, meta):
    """
    Numerical dynamics for TGOV1 governor.

    Args:
        x: numpy array of 2 states [x1, x2]
           x1: valve/gate position
           x2: turbine state
        ports: dict with keys {'omega', 'Pm_ref'}
        meta: dict of governor parameters

    Returns:
        x_dot: numpy array of 2 state derivatives
    """
    # Extract states
    x1, x2 = x

    # Extract ports
    omega = ports.get('omega', 1.0)
    Pm_ref = ports.get('Pm_ref', None)

    # Extract parameters
    R = meta['R']
    T1 = meta['T1']
    T3 = meta['T3']
    Pref = meta['Pref']
    wref = meta['wref']
    VMAX = meta['VMAX']
    VMIN = meta['VMIN']

    # Use Pm_ref from port if provided, otherwise use Pref from metadata
    if Pm_ref is None:
        Pm_ref = Pref

    # Gate command with droop
    gate_cmd = Pm_ref + (wref - omega) / R

    # Apply limits
    gate_limited = np.clip(gate_cmd, VMIN, VMAX)

    # State derivatives
    x_dot = np.zeros(2)

    # Governor dynamics
    x_dot[0] = 10.0 * (gate_limited - x1) / T1  # d(x1)/dt - valve dynamics
    x_dot[1] = 10.0 * (x1 - x2) / T3  # d(x2)/dt - turbine dynamics

    return x_dot


def tgov1_output(x, ports, meta):
    """
    Compute mechanical torque output.

    Args:
        x: numpy array of 2 states [x1, x2]
        ports: dict with keys {'omega'}
        meta: dict of governor parameters

    Returns:
        Tm: mechanical torque
    """
    x1, x2 = x
    omega = ports.get('omega', 1.0)
    T2 = meta['T2']
    T3 = meta['T3']
    Dt = meta['Dt']

    Tm = (T2 / T3) * (x1 - x2) + x2 - Dt * (omega - 1.0)
    return Tm


def build_tgov1_core(gov_data, S_machine=900.0, S_system=100.0):
    """Build TGOV1 governor as DynamicsCore

    Args:
        gov_data: dict with governor parameters
        S_machine: machine base power (MVA)
        S_system: system base power (MVA)

    Returns:
        core: DynamicsCore object with dynamics method
        metadata: dict with additional info
    """
    core = DynamicsCore(label=f'TGOV1_{gov_data["idx"]}', dynamics_fn=tgov1_dynamics)
    
    # Extract and scale parameters
    R_machine = max(gov_data['R'], 0.001)
    R_val = R_machine * (S_system / S_machine)
    T1 = max(gov_data['T1'], 0.001)
    T2 = max(gov_data['T2'], 0.001)
    T3 = max(gov_data['T3'], 0.001)
    Dt = gov_data.get('Dt', 0.0)
    
    wref = 1.0
    Pref = 1.0
    
    # States
    x1, x2 = core.symbols(['x1_gov', 'x2_gov'])
    
    # Parameters
    R_sym, T1_sym, T2_sym, T3_sym = core.symbols(['R_gov', 'T1_gov', 'T2_gov', 'T3_gov'])
    Dt_sym = core.symbols('Dt_gov')
    wref_sym, Pref_sym = core.symbols(['wref_gov', 'Pref_gov'])
    
    # Hamiltonian
    H_gov = (T1_sym/2)*x1**2 + (T3_sym/2)*x2**2
    
    core.add_storages([x1, x2], H_gov)
    
    # Dissipations
    w_x1 = core.symbols('w_x1')
    z_x1 = w_x1 / T1_sym
    
    w_x2 = core.symbols('w_x2')
    z_x2 = w_x2 / T3_sym
    
    core.add_dissipations([w_x1, w_x2], [z_x1, z_x2])
    
    # Ports: Input [omega_in], Output [Tm_out]
    omega_in = core.symbols('omega_in')
    Tm_out = core.symbols('Tm_out')
    
    core.add_ports([omega_in], [Tm_out])
    
    # Parameter substitutions
    core.subs.update({
        R_sym: R_val,
        T1_sym: T1,
        T2_sym: T2,
        T3_sym: T3,
        Dt_sym: Dt,
        wref_sym: wref,
        Pref_sym: Pref
    })
    
    # Metadata
    metadata = {
        'idx': gov_data['idx'],
        'syn': gov_data['syn'],
        'R': R_val,
        'T1': T1,
        'T2': T2,
        'T3': T3,
        'VMAX': gov_data['VMAX'],
        'VMIN': gov_data['VMIN'],
        'Dt': Dt,
        'wref': wref,
        'Pref': Pref
    }

    # Set metadata on core for dynamics computation
    core.set_metadata(metadata)
    
    # Set component interface attributes
    core.n_states = 2
    core.output_fn = tgov1_output
    core.component_type = "governor"
    core.model_name = "TGOV1"

    return core, metadata
