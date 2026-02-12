"""EXST1 Exciter Model - IEEE Type ST1 Static Excitation System

Port-Hamiltonian implementation of IEEE EXST1 exciter.
Based on IEEE Standard 421.5-2005.

Signal path:
    Vt -> [TR lag] -> vm
    Verr = Vref - vm - Vf
    -> [VIMAX/VIMIN limiter] 
    -> [TC/TB lead-lag] 
    -> [KA/(1+sTA) regulator] -> vr
    -> [VRMAX/VRMIN limiter] -> Efd
    Feedback: Efd -> [sKF/(1+sTF) washout] -> Vf
"""

import numpy as np
from numba import njit
from utils.pyphs_core import DynamicsCore


def exst1_dynamics(x, ports, meta):
    """
    EXST1 exciter dynamics with lead-lag compensation and washout feedback.

    States: [vm, vll, vr, vf]
        vm:  Voltage measurement (filtered terminal voltage)
        vll: Lead-lag compensator state
        vr:  Regulator output (with VRMAX/VRMIN limits)
        vf:  Washout feedback state

    Signal flow:
        Vt -> [1/(1+sTR)] -> vm
        Verr = Vref - vm - Vf_out
        Verr -> [VIMAX/VIMIN limits] -> vi
        vi -> [TC/TB lead-lag] -> vll_out
        vll_out -> [KA/(1+sTA)] -> vr (with VRMAX/VRMIN)
        vr -> Efd
        Efd -> [sKF/(1+sTF)] -> Vf_out

    Args:
        x: numpy array [vm, vll, vr, vf]
        ports: dict with 'Vt' (terminal voltage)
        meta: dict of exciter parameters

    Returns:
        x_dot: numpy array of state derivatives
    """
    # Extract states
    vm, vll, vr, vf = x

    # Extract ports
    Vt = ports.get('Vt', 1.0)

    # Extract parameters
    TR = meta['TR']
    VIMAX = meta['VIMAX']
    VIMIN = meta['VIMIN']
    TC = meta['TC']
    TB = meta['TB']
    KA = meta['KA']
    TA = meta['TA']
    VRMAX = meta['VRMAX']
    VRMIN = meta['VRMIN']
    KC = meta['KC']
    KF = meta['KF']
    TF = meta['TF']
    Vref = meta['Vref']
    XadIfd = ports.get('XadIfd', 0.0)  # Field current from generator

    # Initialize derivatives
    x_dot = np.zeros(4)

    # 1. Voltage measurement lag: d(vm)/dt = (Vt - vm)/TR
    x_dot[0] = (Vt - vm) / TR

    # 2. Washout feedback output: Vf_out = KF * (vr - vf) / TF
    Vf_out = KF * (vr - vf) / TF

    # 3. Voltage error
    Verr = Vref - vm - Vf_out

    # 4. Input voltage limiter (VIMAX/VIMIN)
    if Verr > VIMAX:
        vi = VIMAX
    elif Verr < VIMIN:
        vi = VIMIN
    else:
        vi = Verr

    # 5. Lead-lag compensator: (1 + sTC) / (1 + sTB)
    # State equation: d(vll)/dt = (vi - vll) / TB
    # Output: vll_out = vll + TC/TB * (vi - vll)
    # Algebraically: vll_out = (vll + TC/TB * vi - TC/TB * vll)
    #                        = ((1 - TC/TB) * vll + TC/TB * vi)
    #                        = (TB * vll + TC * vi - TC * vll) / TB
    #                        = ((TB - TC) * vll + TC * vi) / TB
    if TB > 1e-6:
        x_dot[1] = (vi - vll) / TB
        vll_out = vll + TC / TB * (vi - vll)
    else:
        # If TB ~ 0, lead-lag becomes gain (TC/TB) * input
        x_dot[1] = 0.0
        vll_out = vi if TC > 1e-6 else 0.0

    # 6. Regulator: d(vr)/dt = (KA * vll_out - vr) / TA
    # With anti-windup for VRMAX/VRMIN limits
    # Dynamic limits include field current compensation
    vr_max_dyn = VRMAX - KC * XadIfd
    vr_min_dyn = VRMIN - KC * XadIfd

    vr_unlimited = (KA * vll_out - vr) / TA

    if vr >= vr_max_dyn and vr_unlimited > 0:
        x_dot[2] = 0.0  # Anti-windup
    elif vr <= vr_min_dyn and vr_unlimited < 0:
        x_dot[2] = 0.0  # Anti-windup
    else:
        x_dot[2] = vr_unlimited

    # 7. Washout feedback filter: d(vf)/dt = (vr - vf) / TF
    x_dot[3] = (vr - vf) / TF

    return x_dot


def exst1_output(x, ports, meta):
    """
    Compute exciter output Efd.

    The output is the regulator state vr, limited by VRMAX/VRMIN.

    Args:
        x: numpy array [vm, vll, vr, vf]
        ports: dict with 'XadIfd' (field current)
        meta: dict of parameters

    Returns:
        Efd: field voltage output (p.u.)
    """
    vm, vll, vr, vf = x

    # Extract parameters for output limits
    VRMAX = meta['VRMAX']
    VRMIN = meta['VRMIN']
    KC = meta['KC']
    XadIfd = ports.get('XadIfd', 0.0)

    # Dynamic limits
    vr_max = VRMAX - KC * XadIfd
    vr_min = VRMIN - KC * XadIfd

    # Apply hard limits to output
    if vr > vr_max:
        Efd = vr_max
    elif vr < vr_min:
        Efd = vr_min
    else:
        Efd = vr

    return Efd


# =============================================================================
# Numba JIT-compiled dynamics (array-based, no dicts)
# =============================================================================

# Meta array indices
_S1M_TR = 0;  _S1M_VIMAX = 1;  _S1M_VIMIN = 2;  _S1M_TC = 3;  _S1M_TB = 4
_S1M_KA = 5;  _S1M_TA = 6;  _S1M_VRMAX = 7;  _S1M_VRMIN = 8
_S1M_KC = 9;  _S1M_KF = 10;  _S1M_TF = 11;  _S1M_Vref = 12
EXST1_META_SIZE = 13

# Ports array indices: [Vt, XadIfd]
_S1P_Vt = 0;  _S1P_XadIfd = 1
EXST1_PORTS_SIZE = 2


def pack_exst1_meta(meta_dict):
    """Pack metadata dict into flat numpy array for JIT."""
    arr = np.zeros(EXST1_META_SIZE)
    arr[_S1M_TR] = meta_dict['TR']
    arr[_S1M_VIMAX] = meta_dict['VIMAX']
    arr[_S1M_VIMIN] = meta_dict['VIMIN']
    arr[_S1M_TC] = meta_dict['TC']
    arr[_S1M_TB] = meta_dict['TB']
    arr[_S1M_KA] = meta_dict['KA']
    arr[_S1M_TA] = meta_dict['TA']
    arr[_S1M_VRMAX] = meta_dict['VRMAX']
    arr[_S1M_VRMIN] = meta_dict['VRMIN']
    arr[_S1M_KC] = meta_dict['KC']
    arr[_S1M_KF] = meta_dict['KF']
    arr[_S1M_TF] = meta_dict['TF']
    arr[_S1M_Vref] = meta_dict['Vref']
    return arr


@njit(cache=True)
def exst1_dynamics_jit(x, ports, meta):
    """JIT-compiled EXST1 dynamics."""
    vm = x[0]; vll = x[1]; vr = x[2]; vf = x[3]
    Vt = ports[0]; XadIfd = ports[1]
    TR = meta[0]; VIMAX = meta[1]; VIMIN = meta[2]; TC = meta[3]; TB = meta[4]
    KA = meta[5]; TA = meta[6]; VRMAX = meta[7]; VRMIN = meta[8]
    KC = meta[9]; KF = meta[10]; TF = meta[11]; Vref = meta[12]

    x_dot = np.zeros(4)
    x_dot[0] = (Vt - vm) / TR
    Vf_out = KF * (vr - vf) / TF
    Verr = Vref - vm - Vf_out

    # Input limiter
    if Verr > VIMAX:
        vi = VIMAX
    elif Verr < VIMIN:
        vi = VIMIN
    else:
        vi = Verr

    # Lead-lag
    if TB > 1e-6:
        x_dot[1] = (vi - vll) / TB
        vll_out = vll + TC / TB * (vi - vll)
    else:
        x_dot[1] = 0.0
        vll_out = vi if TC > 1e-6 else 0.0

    # Regulator with dynamic limits and anti-windup
    vr_max_dyn = VRMAX - KC * XadIfd
    vr_min_dyn = VRMIN - KC * XadIfd
    vr_unlimited = (KA * vll_out - vr) / TA
    if vr >= vr_max_dyn and vr_unlimited > 0.0:
        x_dot[2] = 0.0
    elif vr <= vr_min_dyn and vr_unlimited < 0.0:
        x_dot[2] = 0.0
    else:
        x_dot[2] = vr_unlimited

    x_dot[3] = (vr - vf) / TF
    return x_dot


@njit(cache=True)
def exst1_output_jit(x, ports, meta):
    """JIT-compiled EXST1 output (Efd with dynamic limits)."""
    vr = x[2]
    XadIfd = ports[1]
    VRMAX = meta[7]; VRMIN = meta[8]; KC = meta[9]
    vr_max = VRMAX - KC * XadIfd
    vr_min = VRMIN - KC * XadIfd
    if vr > vr_max:
        return vr_max
    elif vr < vr_min:
        return vr_min
    return vr


def build_exst1_core(exc_data, initial_conditions=None):
    """
    Build EXST1 exciter as DynamicsCore.

    Args:
        exc_data: dict with exciter parameters from JSON
        initial_conditions: dict with initial state values (optional)

    Returns:
        DynamicsCore object configured for EXST1
    """
    # Extract and validate parameters
    TR = max(exc_data.get('TR', 0.02), 0.001)  # Minimum 1ms
    VIMAX = exc_data.get('VIMAX', 99.0)
    VIMIN = exc_data.get('VIMIN', -99.0)
    TC = exc_data.get('TC', 1.0)
    TB = max(exc_data.get('TB', 1.0), 0.001)
    KA = exc_data.get('KA', 50.0)
    TA = max(exc_data.get('TA', 0.05), 0.001)
    VRMAX = exc_data.get('VRMAX', 9999.0)
    VRMIN = exc_data.get('VRMIN', -9999.0)
    KC = exc_data.get('KC', 0.0)
    KF = exc_data.get('KF', 0.01)
    TF = max(exc_data.get('TF', 1.0), 0.001)

    # Reference voltage (typically set during initialization)
    Vref = exc_data.get('Vref', 1.0)

    # Build metadata dictionary
    metadata = {
        'idx': exc_data.get('idx', 'EXST1_1'),
        'name': exc_data.get('name', 'EXST1_1'),
        'syn': exc_data.get('syn', None),
        'TR': TR,
        'VIMAX': VIMAX,
        'VIMIN': VIMIN,
        'TC': TC,
        'TB': TB,
        'KA': KA,
        'TA': TA,
        'VRMAX': VRMAX,
        'VRMIN': VRMIN,
        'KC': KC,
        'KF': KF,
        'TF': TF,
        'Vref': Vref
    }

    # Initialize states: [vm, vll, vr, vf]
    if initial_conditions is not None:
        x0 = np.array([
            initial_conditions.get('vm', 1.0),
            initial_conditions.get('vll', 0.0),
            initial_conditions.get('vr', 1.0),
            initial_conditions.get('vf', 0.0)
        ])
    else:
        # Default initialization for equilibrium
        # Assume Vt = Vref = 1.0, Efd = vr at equilibrium
        vm0 = 1.0
        vll0 = 1.0 / KA  # Equilibrium input to regulator
        vr0 = 1.0  # Nominal field voltage
        vf0 = vr0  # Washout equilibrium
        x0 = np.array([vm0, vll0, vr0, vf0])

    # Create DynamicsCore
    core = DynamicsCore(
        label=f"EXST1_{metadata['idx']}",
        dynamics_fn=exst1_dynamics
    )

    # Set metadata and component attributes
    core.set_metadata(metadata)
    core.n_states = 4
    core.output_fn = exst1_output
    def _exst1_init(Efd_eq, V_mag, **kwargs):
        x0, Vref_eq = compute_initial_states(V_mag, Efd_eq, metadata)
        metadata['Vref'] = Vref_eq  # Update so dynamics uses correct reference
        return x0
    core.init_fn = _exst1_init
    core.component_type = "exciter"
    core.model_name = "EXST1"

    return core, metadata


def compute_initial_states(Vt, Efd_desired, params):
    """
    Compute equilibrium initial states for EXST1 given terminal voltage and desired field voltage.

    At equilibrium, all derivatives are zero, so:
        vm = Vt
        vf = vr (washout equilibrium)
        Vf_out = 0 (washout output is zero at equilibrium)
        Verr = Vref - vm = constant
        vll = vi (lead-lag equilibrium, d(vll)/dt = 0)
        vr = Efd_desired

    Args:
        Vt: Terminal voltage (p.u.)
        Efd_desired: Desired field voltage (p.u.)
        params: EXST1 parameter dict

    Returns:
        x0: numpy array [vm, vll, vr, vf]
        Vref: Required reference voltage
    """
    KA = params.get('KA', 50.0)
    TC = params.get('TC', 1.0)
    TB = max(params.get('TB', 1.0), 0.001)
    VIMAX = params.get('VIMAX', 99.0)
    VIMIN = params.get('VIMIN', -99.0)

    # State equilibrium values
    vm = Vt
    vr = Efd_desired
    vf = vr  # Washout equilibrium (d(vf)/dt = 0 => vf = vr)

    # Work backwards through the regulator chain
    # At equilibrium: d(vr)/dt = 0 => KA * vll_out = vr
    vll_out = vr / KA

    # For lead-lag at equilibrium: d(vll)/dt = 0
    # d(vll)/dt = (vi - vll) / TB = 0 => vll = vi
    # vll_out = vll + (TC/TB) * (vi - vll) = vi + (TC/TB) * 0 = vi
    # Therefore: vi = vll_out
    vi = vll_out
    vll = vi  # At equilibrium

    # vi comes from limiting Verr
    # Assuming no limiting at equilibrium: vi = Verr
    Verr = vi

    # Verr = Vref - vm - Vf_out, and Vf_out = 0 at equilibrium
    # Therefore: Vref = Verr + vm
    Vref = Verr + vm

    x0 = np.array([vm, vll, vr, vf])

    return x0, Vref


# Example usage and testing
if __name__ == "__main__":
    # Test EXST1 initialization
    test_params = {
        'idx': 'EXST1_test',
        'name': 'EXST1_test',
        'syn': 'GENROU_1',
        'TR': 0.02,
        'VIMAX': 99.0,
        'VIMIN': -99.0,
        'TC': 1.0,
        'TB': 1.0,
        'KA': 50.0,
        'TA': 0.02,
        'VRMAX': 9999.0,
        'VRMIN': -9999.0,
        'KC': 0.0,
        'KF': 0.01,
        'TF': 1.0,
        'Vref': 1.05
    }

    # Build exciter
    exciter = build_exst1_core(test_params)

    # Test dynamics at nominal conditions
    test_ports = {'Vt': 1.0, 'XadIfd': 0.0}
    x_dot = exciter.dynamics_fn(exciter.states, test_ports, exciter.meta)
    Efd = exciter.output_fn(exciter.states, test_ports, exciter.meta)

    print("EXST1 Exciter Test")
    print(f"States: {exciter.states}")
    print(f"Derivatives: {x_dot}")
    print(f"Output Efd: {Efd}")
    print(f"Max |dx/dt|: {np.max(np.abs(x_dot))}")
