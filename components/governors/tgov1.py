"""TGOV1 Governor Model"""
import sys
sys.path.insert(0, '/home/claude')
from utils.pyphs_core import DynamicsCore
import numpy as np
from numba import njit


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
    deadband = meta.get('deadband', 0.0)  # Frequency deadband (for equilibrium tests)

    # Use Pm_ref from port if provided, otherwise use Pref from metadata
    if Pm_ref is None:
        Pm_ref = Pref

    # Frequency error with deadband
    freq_error = wref - omega
    if abs(freq_error) < deadband:
        freq_error = 0.0  # No response within deadband

    # Gate command with droop
    gate_cmd = Pm_ref + freq_error / R

    # Apply limits
    gate_limited = np.clip(gate_cmd, VMIN, VMAX)

    # State derivatives
    x_dot = np.zeros(2)

    # Governor dynamics (standard TGOV1 transfer functions)
    x_dot[0] = (gate_limited - x1) / T1  # d(x1)/dt - valve dynamics
    x_dot[1] = (x1 - x2) / T3  # d(x2)/dt - turbine dynamics

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


# =============================================================================
# Numba JIT-compiled dynamics (array-based, no dicts)
# =============================================================================

# Meta array indices
_TM_R = 0;  _TM_T1 = 1;  _TM_T2 = 2;  _TM_T3 = 3
_TM_Pref = 4;  _TM_wref = 5;  _TM_VMAX = 6;  _TM_VMIN = 7
_TM_deadband = 8;  _TM_Dt = 9
TGOV1_META_SIZE = 10

# Ports array indices
_TP_omega = 0
TGOV1_PORTS_SIZE = 1


def pack_tgov1_meta(meta_dict):
    """Pack metadata dict into flat numpy array for JIT."""
    arr = np.zeros(TGOV1_META_SIZE)
    arr[_TM_R] = meta_dict['R']
    arr[_TM_T1] = meta_dict['T1']
    arr[_TM_T2] = meta_dict['T2']
    arr[_TM_T3] = meta_dict['T3']
    arr[_TM_Pref] = meta_dict['Pref']
    arr[_TM_wref] = meta_dict['wref']
    arr[_TM_VMAX] = meta_dict['VMAX']
    arr[_TM_VMIN] = meta_dict['VMIN']
    arr[_TM_deadband] = meta_dict.get('deadband', 0.0)
    arr[_TM_Dt] = meta_dict.get('Dt', 0.0)
    return arr


@njit(cache=True)
def tgov1_dynamics_jit(x, ports, meta):
    """JIT-compiled TGOV1 dynamics."""
    x1 = x[0]; x2 = x[1]
    omega = ports[0]
    R = meta[0]; T1 = meta[1]; T3 = meta[3]
    Pref = meta[4]; wref = meta[5]; VMAX = meta[6]; VMIN = meta[7]
    deadband = meta[8]

    freq_error = wref - omega
    if abs(freq_error) < deadband:
        freq_error = 0.0

    gate_cmd = Pref + freq_error / R
    gate_limited = min(max(gate_cmd, VMIN), VMAX)

    x_dot = np.zeros(2)
    x_dot[0] = (gate_limited - x1) / T1
    x_dot[1] = (x1 - x2) / T3
    return x_dot


@njit(cache=True)
def tgov1_output_jit(x, ports, meta):
    """JIT-compiled TGOV1 output (mechanical torque)."""
    x1 = x[0]; x2 = x[1]
    omega = ports[0]
    T2 = meta[2]; T3 = meta[3]; Dt = meta[9]
    return (T2 / T3) * (x1 - x2) + x2 - Dt * (omega - 1.0)


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
    base_ratio = S_machine / S_system  # convert machine-base pu to system-base pu
    R_machine = max(gov_data['R'], 0.001)
    R_val = R_machine * (S_system / S_machine)
    T1 = max(gov_data['T1'], 0.001)
    T2 = max(gov_data['T2'], 0.001)
    T3 = max(gov_data['T3'], 0.001)
    Dt = gov_data.get('Dt', 0.0)
    deadband = gov_data.get('deadband', 0.0)  # Frequency deadband (pu)

    # Governor limits are provided on machine base; scale to system base so they
    # do not clip realistic mechanical power on heavily loaded machines.
    VMAX_sys = gov_data['VMAX'] * base_ratio
    VMIN_sys = gov_data['VMIN'] * base_ratio

    wref = 1.0
    Pref = base_ratio  # default reference ~1 pu on machine base
    
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
        'VMAX': VMAX_sys,
        'VMIN': VMIN_sys,
        'Dt': Dt,
        'deadband': deadband,
        'wref': wref,
        'Pref': Pref
    }

    # Set metadata on core for dynamics computation
    core.set_metadata(metadata)
    
    # Set component interface attributes
    core.n_states = 2
    core.output_fn = tgov1_output
    core.init_fn = lambda Pm_eq, **kwargs: np.array([Pm_eq, Pm_eq])
    core.component_type = "governor"
    core.model_name = "TGOV1"
    
    # Override symbolic dynamics with numerical implementation (includes deadband)
    core._dynamics_fn = tgov1_dynamics

    return core, metadata
