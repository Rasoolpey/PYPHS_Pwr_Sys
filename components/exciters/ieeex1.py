"""IEEEX1 Exciter Model - IEEE Type DC1A Excitation System

Port-Hamiltonian implementation of IEEE IEEEX1 exciter.
Based on IEEE Standard 421.5-2005 Type DC1A.

This model is similar to EXDC2 but with voltage-dependent limits:
    VRMAX_eff = VRMAX * Vt
    VRMIN_eff = VRMIN * Vt

Signal path:
    Vt -> [TR lag] -> vm
    Verr = Vref - vm - Vf
    -> [TC/TB lead-lag] 
    -> [KA/(1+sTA) regulator with V-dependent limits] -> vr
    -> [Exciter with saturation] -> vp
    -> [Speed compensation] -> Efd
    Feedback: vp -> [sKF/(1+sTF) washout] -> Vf
"""

import numpy as np
from utils.pyphs_core import DynamicsCore


def ieeex1_dynamics(x, ports, meta):
    """
    IEEEX1 exciter dynamics with exciter saturation and voltage-dependent limits.

    States: [vm, vll, vr, vp, vf]
        vm:  Voltage measurement (filtered terminal voltage)
        vll: Lead-lag compensator state
        vr:  Regulator output (after anti-windup lag)
        vp:  Exciter output (before speed compensation)
        vf:  Washout feedback state

    Signal flow:
        Vt -> [1/(1+sTR)] -> vm
        Verr = Vref - vm - Vf_out
        Verr -> [TC/TB lead-lag] -> vll_out
        vll_out -> [KA/(1+sTA)] -> vr (with V-dependent limits)
        vr -> [Exciter with saturation] -> vp
        vp -> [Speed compensation] -> Efd
        vp -> [sKF/(1+sTF)] -> Vf_out

    Args:
        x: numpy array [vm, vll, vr, vp, vf]
        ports: dict with 'Vt' (terminal voltage), 'omega' (speed)
        meta: dict of exciter parameters

    Returns:
        x_dot: numpy array of state derivatives
    """
    # Extract states
    vm, vll, vr, vp, vf = x

    # Extract ports
    Vt = ports.get('Vt', 1.0)
    omega = ports.get('omega', 1.0)

    # Extract parameters
    TR = meta['TR']
    TC = meta['TC']
    TB = meta['TB']
    KA = meta['KA']
    TA = meta['TA']
    VRMAX = meta['VRMAX']
    VRMIN = meta['VRMIN']
    TE = meta['TE']
    KE = meta['KE']
    E1 = meta['E1']
    SE1 = meta['SE1']
    E2 = meta['E2']
    SE2 = meta['SE2']
    KF1 = meta['KF1']
    TF1 = meta['TF1']
    Vref = meta['Vref']

    # Initialize derivatives
    x_dot = np.zeros(5)

    # 1. Voltage measurement lag: d(vm)/dt = (Vt - vm)/TR
    x_dot[0] = (Vt - vm) / TR if TR > 1e-6 else 0.0

    # 2. Washout feedback: d(vf)/dt = (vp - vf) / TF1
    # Feedback output: Vf_out = KF1 * (vp - vf) / TF1
    if TF1 > 1e-6:
        x_dot[4] = (vp - vf) / TF1
        Vf_out = KF1 * (vp - vf) / TF1
    else:
        x_dot[4] = 0.0
        Vf_out = 0.0

    # 3. Voltage error
    Verr = Vref - vm - Vf_out

    # 4. Lead-lag compensator: (1 + sTC) / (1 + sTB)
    # State equation: d(vll)/dt = (Verr - vll) / TB
    # Output: vll_out = vll + TC/TB * (Verr - vll)
    if TB > 1e-6:
        x_dot[1] = (Verr - vll) / TB
        vll_out = vll + TC / TB * (Verr - vll)
    else:
        x_dot[1] = 0.0
        vll_out = Verr if TC > 1e-6 else 0.0

    # 5. Anti-windup lag regulator: KA / (1 + sTA)
    # State equation: d(vr)/dt = (KA * vll_out - vr) / TA
    # With voltage-dependent anti-windup limits
    vr_max = VRMAX * Vt
    vr_min = VRMIN * Vt
    
    vr_unlimited = KA * vll_out
    
    if TA > 1e-6:
        if vr >= vr_max and vr_unlimited > vr:
            x_dot[2] = 0.0  # Anti-windup at upper limit
        elif vr <= vr_min and vr_unlimited < vr:
            x_dot[2] = 0.0  # Anti-windup at lower limit
        else:
            x_dot[2] = (vr_unlimited - vr) / TA
    else:
        x_dot[2] = 0.0

    # 6. Exciter with saturation: d(vp)/dt = (vr - KE*vp - Se*vp) / TE
    # Exciter saturation function
    Se = compute_saturation(vp, E1, SE1, E2, SE2)
    
    if TE > 1e-6:
        x_dot[3] = (vr - KE * vp - Se * vp) / TE
    else:
        x_dot[3] = 0.0

    return x_dot


def compute_saturation(vp, E1, SE1, E2, SE2):
    """
    Compute exciter saturation function Se(vp).
    
    Uses exponential saturation model:
    Se(vp) = B * (vp - A)^2 / vp  for vp > A
    Se(vp) = 0                     for vp <= A
    
    Where A and B are computed from saturation data points (E1, SE1) and (E2, SE2).
    
    Args:
        vp: Exciter voltage
        E1, SE1: First saturation point
        E2, SE2: Second saturation point
    
    Returns:
        Se: Saturation function value
    """
    vp_abs = abs(vp)
    
    # Disable saturation if all parameters are zero or one
    if E1 <= 1e-6 or E2 <= 1e-6 or SE1 <= 1e-6:
        return 0.0
    
    # Compute saturation coefficients A and B
    # From two points: SE1 = B*(E1-A)^2/E1 and SE2 = B*(E2-A)^2/E2
    # Solving: A and B
    try:
        # Quadratic saturation model
        # SE1 * E1 = B * (E1 - A)^2
        # SE2 * E2 = B * (E2 - A)^2
        # Ratio: SE1*E1 / (SE2*E2) = (E1-A)^2 / (E2-A)^2
        ratio = SE1 * E1 / (SE2 * E2) if SE2 * E2 > 1e-9 else 0.0
        sqrt_ratio = np.sqrt(ratio) if ratio > 0 else 0.0
        
        # A = (E1 - sqrt_ratio * E2) / (1 - sqrt_ratio)
        if abs(1 - sqrt_ratio) > 1e-6:
            A = (E1 - sqrt_ratio * E2) / (1 - sqrt_ratio)
        else:
            A = E1 * 0.5  # Fallback
        
        # B = SE1 * E1 / (E1 - A)^2
        if abs(E1 - A) > 1e-6:
            B = SE1 * E1 / ((E1 - A) ** 2)
        else:
            B = 0.0
        
        # Apply saturation
        if vp_abs > A:
            Se = B * ((vp_abs - A) ** 2) / vp_abs
        else:
            Se = 0.0
            
    except (ZeroDivisionError, ValueError):
        Se = 0.0
    
    return Se


def ieeex1_output(x, ports, meta):
    """
    Compute field voltage output with speed compensation.

    Args:
        x: numpy array [vm, vll, vr, vp, vf]
        ports: dict with 'omega' (rotor speed)
        meta: dict of parameters

    Returns:
        Efd: field voltage output (p.u.)
    """
    vm, vll, vr, vp, vf = x
    omega = ports.get('omega', 1.0)
    
    # Field voltage with speed compensation
    # Efd = vp * omega
    Efd = vp * omega
    
    return Efd


def compute_initial_states(Vt, Efd_desired, omega, params):
    """
    Compute equilibrium initial states for IEEEX1.

    At equilibrium with omega=1.0:
        vm = Vt
        vf = vp (washout equilibrium)
        Vf_out = 0 (washout output is zero)
        vp = Efd_desired / omega
        Se = saturation at vp
        vr = (KE + Se) * vp
        vll = vr / KA
        Vref = vm + vll / KA

    Args:
        Vt: Terminal voltage (p.u.)
        Efd_desired: Desired field voltage (p.u.)
        omega: Rotor speed (p.u.)
        params: IEEEX1 parameter dict

    Returns:
        x0: numpy array [vm, vll, vr, vp, vf]
        Vref: Required reference voltage
    """
    KA = params.get('KA', 40.0)
    KE = params.get('KE', 1.0)
    TC = params.get('TC', 1.0)
    TB = params.get('TB', 1.0)
    E1 = params.get('E1', 0.0)
    SE1 = params.get('SE1', 0.0)
    E2 = params.get('E2', 1.0)
    SE2 = params.get('SE2', 1.0)

    # State equilibrium values
    vm = Vt
    vp = Efd_desired / omega if omega > 0.1 else Efd_desired
    vf = vp  # Washout equilibrium
    
    # Compute saturation at equilibrium
    Se = compute_saturation(vp, E1, SE1, E2, SE2)
    
    # Work backwards through the regulator
    vr = (KE + Se) * vp
    
    # vr = KA * vll_out => vll_out = vr / KA
    vll_out = vr / KA
    
    # At equilibrium: vll = vll_out (lead-lag equilibrium when TC=TB)
    # For general case: vll = vll_out
    vll = vll_out
    
    # Verr = vll_out (no feedback at equilibrium)
    Verr = vll_out
    
    # Vref = Verr + vm (Vf_out = 0 at equilibrium)
    Vref = Verr + vm

    x0 = np.array([vm, vll, vr, vp, vf])

    return x0, Vref


def build_ieeex1_core(exc_data, initial_conditions=None):
    """
    Build IEEEX1 exciter as DynamicsCore.

    Args:
        exc_data: dict with exciter parameters from JSON
        initial_conditions: dict with initial state values (optional)

    Returns:
        DynamicsCore object configured for IEEEX1
    """
    # Extract and validate parameters
    TR = max(exc_data.get('TR', 0.01), 0.001)  # Minimum 1ms
    TC = exc_data.get('TC', 1.0)
    TB = max(exc_data.get('TB', 1.0), 0.001)
    KA = exc_data.get('KA', 40.0)
    TA = max(exc_data.get('TA', 0.04), 0.001)
    VRMAX = exc_data.get('VRMAX', 7.3)
    VRMIN = exc_data.get('VRMIN', -7.3)
    TE = max(exc_data.get('TE', 0.8), 0.001)
    KE = exc_data.get('KE', 1.0)
    KF1 = exc_data.get('KF1', 0.03)
    TF1 = max(exc_data.get('TF1', 1.0), 0.001)
    E1 = exc_data.get('E1', 0.0)
    SE1 = exc_data.get('SE1', 0.0)
    E2 = exc_data.get('E2', 1.0)
    SE2 = exc_data.get('SE2', 1.0)

    # Reference voltage (typically set during initialization)
    Vref = exc_data.get('Vref', 1.0)

    # Build metadata dictionary
    metadata = {
        'idx': exc_data.get('idx', 'IEEEX1_1'),
        'name': exc_data.get('name', 'IEEEX1_1'),
        'syn': exc_data.get('syn', None),
        'TR': TR,
        'TC': TC,
        'TB': TB,
        'KA': KA,
        'TA': TA,
        'VRMAX': VRMAX,
        'VRMIN': VRMIN,
        'TE': TE,
        'KE': KE,
        'KF1': KF1,
        'TF1': TF1,
        'E1': E1,
        'SE1': SE1,
        'E2': E2,
        'SE2': SE2,
        'Vref': Vref
    }

    # Initialize states: [vm, vll, vr, vp, vf]
    if initial_conditions is not None:
        x0 = np.array([
            initial_conditions.get('vm', 1.0),
            initial_conditions.get('vll', 0.0),
            initial_conditions.get('vr', 1.0),
            initial_conditions.get('vp', 1.0),
            initial_conditions.get('vf', 1.0)
        ])
    else:
        # Default initialization for equilibrium at nominal voltage
        vm0 = 1.0
        vp0 = 1.0
        vf0 = vp0
        Se0 = compute_saturation(vp0, E1, SE1, E2, SE2)
        vr0 = (KE + Se0) * vp0
        vll0 = vr0 / KA
        x0 = np.array([vm0, vll0, vr0, vp0, vf0])

    # Create DynamicsCore
    core = DynamicsCore(
        label=f"IEEEX1_{metadata['idx']}",
        dynamics_fn=ieeex1_dynamics
    )

    # Set metadata and component attributes
    core.set_metadata(metadata)
    core.n_states = 5
    core.output_fn = ieeex1_output
    def _ieeex1_init(Efd_eq, V_mag, **kwargs):
        x0, Vref = compute_initial_states(
            V_mag, Efd_eq, kwargs.get('omega', 1.0), metadata
        )
        metadata['Vref'] = Vref
        return x0

    core.init_fn = _ieeex1_init
    core.component_type = "exciter"
    core.model_name = "IEEEX1"

    return core, metadata


# Example usage and testing
if __name__ == "__main__":
    # Test IEEEX1 initialization
    test_params = {
        'idx': 'IEEEX1_test',
        'name': 'IEEEX1_test',
        'syn': 'GENROU_1',
        'TR': 0.01,
        'TC': 1.0,
        'TB': 1.0,
        'KA': 40.0,
        'TA': 0.04,
        'VRMAX': 7.3,
        'VRMIN': -7.3,
        'TE': 0.8,
        'KE': 1.0,
        'KF1': 0.03,
        'TF1': 1.0,
        'E1': 3.0,
        'SE1': 0.1,
        'E2': 4.0,
        'SE2': 0.3,
        'Vref': 1.05
    }

    core, metadata = build_ieeex1_core(test_params)
    print(f"IEEEX1 core built: {core.label}")
    print(f"  States: {core.n_states}")
    print(f"  Model: {core.model_name}")
    
    # Test dynamics at equilibrium
    x0, Vref = compute_initial_states(1.0, 1.2, 1.0, metadata)
    print(f"\nEquilibrium states: {x0}")
    print(f"Required Vref: {Vref:.4f}")
    
    ports = {'Vt': 1.0, 'omega': 1.0}
    metadata['Vref'] = Vref
    dx = ieeex1_dynamics(x0, ports, metadata)
    print(f"Derivatives at equilibrium: {dx}")
    print(f"Max |dx/dt|: {np.max(np.abs(dx)):.2e}")
