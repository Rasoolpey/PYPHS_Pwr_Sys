"""IEEEG1 Governor Model - IEEE Type G1 Steam Turbine Governor

Port-Hamiltonian implementation of IEEE IEEEG1 governor.
Based on IEEE Standard 421.5-2005.

Multi-stage steam turbine model with:
- Speed droop control with lead-lag compensation
- Valve speed controller with rate limits
- Four turbine/reheater stages (HP and LP outputs)
- Power fraction distribution across stages

Signal path:
    omega_err = wref - omega
    -> [K*T2/(1+sT1) lead-lag] 
    -> [1/T3 valve speed] with UO/UC rate limits
    -> [integrator] valve position (PMIN to PMAX)
    -> [1/(1+sT4)] -> L4 (inlet/steam bowl)
    -> [1/(1+sT5)] -> L5 (reheater)
    -> [1/(1+sT6)] -> L6 (3rd stage)
    -> [1/(1+sT7)] -> L7 (4th stage/2nd reheater)
    -> Pm = K1*L4 + K3*L5 + K5*L6 + K7*L7 (HP output)
         + K2*L4 + K4*L5 + K6*L6 + K8*L7 (LP output)
"""

import numpy as np
from utils.pyphs_core import DynamicsCore


def ieeeg1_dynamics(x, ports, meta):
    """
    IEEEG1 governor dynamics for multi-stage steam turbine.

    States: [xll, vpos, x4, x5, x6, x7]
        xll:  Lead-lag filter state
        vpos: Valve position (integrated from valve speed)
        x4:   First lag state (inlet piping/steam bowl, T4)
        x5:   Second lag state (reheater, T5)
        x6:   Third lag state (T6)
        x7:   Fourth lag state (2nd reheater, T7)

    Args:
        x: numpy array [xll, vpos, x4, x5, x6, x7]
        ports: dict with 'omega' (generator speed), 'Pm_aux' (auxiliary power)
        meta: dict of governor parameters

    Returns:
        x_dot: numpy array of state derivatives
    """
    # Extract states
    xll, vpos, x4, x5, x6, x7 = x

    # Extract ports
    omega = ports.get('omega', 1.0)
    Pm_aux = ports.get('Pm_aux', 0.0)  # Auxiliary power signal

    # Extract parameters
    K = meta['K']
    T1 = meta['T1']
    T2 = meta['T2']
    T3 = meta['T3']
    UO = meta['UO']
    UC = meta['UC']
    PMAX = meta['PMAX']
    PMIN = meta['PMIN']
    T4 = meta['T4']
    T5 = meta['T5']
    T6 = meta['T6']
    T7 = meta['T7']
    wref = meta['wref']
    Pref = meta.get('Pref', 0.0)  # Power reference setpoint

    # Initialize derivatives
    x_dot = np.zeros(6)

    # 1. Speed deviation
    wd = wref - omega

    # 2. Lead-lag compensator: T2/T1
    # State equation: d(xll)/dt = (wd - xll) / T1
    # Output: LL_y = K * (wd + (T2 - T1)/T1 * xll)
    if T1 > 1e-6:
        x_dot[0] = (wd - xll) / T1
        LL_y = K * (wd + (T2 - T1) * xll / T1)
    else:
        x_dot[0] = 0.0
        LL_y = K * T2 * wd

    # 3. Valve speed calculation with power reference
    # At equilibrium: LL_y=0, Pm_aux=0, vpos=Pref => vs=0
    vs = (LL_y + Pm_aux + Pref - vpos) / T3

    # 4. Valve speed rate limits (UO = max opening, UC = max closing)
    if vs > UO:
        vsl = UO
    elif vs < UC:
        vsl = UC
    else:
        vsl = vs

    # 5. Valve position integrator with anti-windup
    # d(vpos)/dt = vsl
    vpos_unlimited = vsl
    
    if vpos >= PMAX and vsl > 0:
        x_dot[1] = 0.0  # Anti-windup at upper limit
    elif vpos <= PMIN and vsl < 0:
        x_dot[1] = 0.0  # Anti-windup at lower limit
    else:
        x_dot[1] = vpos_unlimited

    # 6. First lag (inlet piping/steam bowl): d(x4)/dt = (vpos - x4) / T4
    if T4 > 1e-6:
        x_dot[2] = (vpos - x4) / T4
    else:
        x_dot[2] = 0.0
        x4 = vpos  # Bypass if T4 ~ 0

    # 7. Second lag (reheater): d(x5)/dt = (x4 - x5) / T5
    if T5 > 1e-6:
        x_dot[3] = (x4 - x5) / T5
    else:
        x_dot[3] = 0.0
        x5 = x4

    # 8. Third lag: d(x6)/dt = (x5 - x6) / T6
    if T6 > 1e-6:
        x_dot[4] = (x5 - x6) / T6
    else:
        x_dot[4] = 0.0
        x6 = x5

    # 9. Fourth lag (2nd reheater): d(x7)/dt = (x6 - x7) / T7
    if T7 > 1e-6:
        x_dot[5] = (x6 - x7) / T7
    else:
        x_dot[5] = 0.0
        x7 = x6

    return x_dot


def ieeeg1_output(x, ports, meta):
    """
    Compute turbine mechanical power output.

    Power is distributed across four stages with fractions K1-K8:
        Pm_HP = K1*L4 + K3*L5 + K5*L6 + K7*L7  (High pressure)
        Pm_LP = K2*L4 + K4*L5 + K6*L6 + K8*L7  (Low pressure)
        Pm_total = Pm_HP + Pm_LP

    Args:
        x: numpy array [xll, vpos, x4, x5, x6, x7]
        ports: dict (not directly used for output)
        meta: dict with K1-K8 power fractions

    Returns:
        Pm: Total mechanical power output (p.u.)
    """
    xll, vpos, x4, x5, x6, x7 = x

    # Extract normalized power fractions
    K1n = meta['K1n']
    K2n = meta['K2n']
    K3n = meta['K3n']
    K4n = meta['K4n']
    K5n = meta['K5n']
    K6n = meta['K6n']
    K7n = meta['K7n']
    K8n = meta['K8n']

    # High-pressure output
    Pm_HP = K1n * x4 + K3n * x5 + K5n * x6 + K7n * x7

    # Low-pressure output
    Pm_LP = K2n * x4 + K4n * x5 + K6n * x6 + K8n * x7

    # Total mechanical power
    Pm = Pm_HP + Pm_LP

    return Pm


def build_ieeeg1_core(gov_data, S_machine=900.0, S_system=100.0, initial_conditions=None):
    """
    Build IEEEG1 governor as DynamicsCore.

    Args:
        gov_data: dict with governor parameters from JSON
        initial_conditions: dict with initial state values (optional)

    Returns:
        DynamicsCore object configured for IEEEG1
    """
    # Extract and validate parameters
    K = gov_data.get('K', 20.0)
    T1 = max(gov_data.get('T1', 0.1), 0.001)
    T2 = gov_data.get('T2', 0.0)
    T3 = max(gov_data.get('T3', 0.2), 0.001)
    UO = gov_data.get('UO', 1.0)
    UC = gov_data.get('UC', -1.0)
    PMAX = gov_data.get('PMAX', 0.95)
    PMIN = gov_data.get('PMIN', 0.0)

    T4 = max(gov_data.get('T4', 0.1), 0.001)
    T5 = max(gov_data.get('T5', 0.5), 0.001)
    T6 = max(gov_data.get('T6', 0.5), 0.001)
    T7 = max(gov_data.get('T7', 0.05), 0.001)

    # Power fractions (K1-K8)
    K1 = gov_data.get('K1', 0.0)
    K2 = gov_data.get('K2', 0.0)
    K3 = gov_data.get('K3', 0.0)
    K4 = gov_data.get('K4', 0.0)
    K5 = gov_data.get('K5', 0.3)
    K6 = gov_data.get('K6', 0.0)
    K7 = gov_data.get('K7', 0.7)
    K8 = gov_data.get('K8', 0.0)

    # Normalize K1-K8 to sum to 1.0
    K_sum = K1 + K2 + K3 + K4 + K5 + K6 + K7 + K8
    if K_sum < 1e-6:
        # Default distribution if all zeros
        K1, K7 = 0.3, 0.7
        K_sum = 1.0

    K_coeff = 1.0 / K_sum
    K1n = K1 * K_coeff
    K2n = K2 * K_coeff
    K3n = K3 * K_coeff
    K4n = K4 * K_coeff
    K5n = K5 * K_coeff
    K6n = K6 * K_coeff
    K7n = K7 * K_coeff
    K8n = K8 * K_coeff

    # Reference speed and initial power
    wref = gov_data.get('wref0', 1.0)
    Tm0 = gov_data.get('Tm0', 0.8)  # Initial mechanical power

    # Build metadata dictionary
    metadata = {
        'idx': gov_data.get('idx', 'IEEEG1_1'),
        'name': gov_data.get('name', 'IEEEG1_1'),
        'syn': gov_data.get('syn', None),
        'K': K,
        'T1': T1,
        'T2': T2,
        'T3': T3,
        'UO': UO,
        'UC': UC,
        'PMAX': PMAX,
        'PMIN': PMIN,
        'T4': T4,
        'T5': T5,
        'T6': T6,
        'T7': T7,
        'K1': K1,
        'K2': K2,
        'K3': K3,
        'K4': K4,
        'K5': K5,
        'K6': K6,
        'K7': K7,
        'K8': K8,
        'K1n': K1n,
        'K2n': K2n,
        'K3n': K3n,
        'K4n': K4n,
        'K5n': K5n,
        'K6n': K6n,
        'K7n': K7n,
        'K8n': K8n,
        'wref': wref,
        'Tm0': Tm0
    }

    # Initialize states: [xll, vpos, x4, x5, x6, x7]
    if initial_conditions is not None:
        x0 = np.array([
            initial_conditions.get('xll', 0.0),
            initial_conditions.get('vpos', Tm0),
            initial_conditions.get('x4', Tm0),
            initial_conditions.get('x5', Tm0),
            initial_conditions.get('x6', Tm0),
            initial_conditions.get('x7', Tm0)
        ])
    else:
        # Default initialization for equilibrium
        # At equilibrium: omega = wref, all states = Tm0
        xll0 = 0.0  # No speed deviation
        vpos0 = Tm0  # Valve position at initial power
        x4_0 = Tm0
        x5_0 = Tm0
        x6_0 = Tm0
        x7_0 = Tm0
        x0 = np.array([xll0, vpos0, x4_0, x5_0, x6_0, x7_0])

    # Create DynamicsCore
    core = DynamicsCore(
        label=f"IEEEG1_{metadata['idx']}",
        dynamics_fn=ieeeg1_dynamics
    )

    # Set metadata and component attributes
    core.set_metadata(metadata)
    core.n_states = 6
    core.output_fn = ieeeg1_output
    core.init_fn = lambda Pm_eq, **kwargs: np.array([0.0, Pm_eq, Pm_eq, Pm_eq, Pm_eq, Pm_eq])
    core.component_type = "governor"
    core.model_name = "IEEEG1"

    return core, metadata


def compute_initial_states(omega, Pm_desired, params):
    """
    Compute equilibrium initial states for IEEEG1.

    At equilibrium with omega = wref:
        All derivatives = 0
        xll = 0 (no speed deviation)
        vpos = x4 = x5 = x6 = x7 = Pm_desired

    Args:
        omega: Generator speed (should be wref for equilibrium)
        Pm_desired: Desired mechanical power output
        params: IEEEG1 parameter dict

    Returns:
        x0: numpy array [xll, vpos, x4, x5, x6, x7]
    """
    wref = params.get('wref', 1.0)

    # Verify equilibrium condition
    if abs(omega - wref) > 1e-6:
        print(f"Warning: omega={omega} != wref={wref}. Not at equilibrium.")

    # All states equal to desired power at equilibrium
    xll = 0.0
    vpos = Pm_desired
    x4 = Pm_desired
    x5 = Pm_desired
    x6 = Pm_desired
    x7 = Pm_desired

    x0 = np.array([xll, vpos, x4, x5, x6, x7])

    return x0


# Example usage and testing
if __name__ == "__main__":
    # Test IEEEG1 initialization
    test_params = {
        'idx': 'IEEEG1_test',
        'name': 'IEEEG1_test',
        'syn': 'GENROU_1',
        'K': 20.0,
        'T1': 0.1,
        'T2': 0.0,
        'T3': 0.2,
        'UO': 1.0,
        'UC': -1.0,
        'PMAX': 0.95,
        'PMIN': 0.0,
        'T4': 0.1,
        'T5': 0.0,
        'T6': 0.0,
        'T7': 8.72,
        'K1': 0.0,
        'K2': 0.0,
        'K3': 0.0,
        'K4': 0.0,
        'K5': 0.3,
        'K6': 0.0,
        'K7': 0.7,
        'K8': 0.0,
        'wref0': 1.0,
        'Tm0': 0.8
    }

    # Build governor
    governor = build_ieeeg1_core(test_params)

    # Test dynamics at nominal conditions
    test_ports = {'omega': 1.0, 'Pm_aux': 0.0}
    x_dot = governor.dynamics_fn(governor.states, test_ports, governor.meta)
    Pm = governor.output_fn(governor.states, test_ports, governor.meta)

    print("IEEEG1 Governor Test")
    print(f"States: {governor.states}")
    print(f"Derivatives: {x_dot}")
    print(f"Output Pm: {Pm}")
    print(f"Max |dx/dt|: {np.max(np.abs(x_dot))}")
