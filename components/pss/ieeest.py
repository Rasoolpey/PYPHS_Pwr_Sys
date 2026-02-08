"""IEEEST PSS Model - IEEE Type ST Power System Stabilizer

Port-Hamiltonian implementation of IEEE IEEEST PSS.
Based on IEEE Standard 421.5-2005.

Signal path:
    Input signal (MODE selection)
    -> [2nd order lag A1/A2] 
    -> [2nd order lead-lag A3-A6]
    -> [lead-lag T1/T2]
    -> [lead-lag T3/T4]
    -> [gain KS]
    -> [washout T5/T6]
    -> [LSMAX/LSMIN limits]
    -> Vss (stabilizing signal to exciter)

Input modes:
    1: Rotor speed deviation (ω - 1)
    2: Bus frequency deviation (f - 1)
    3: Gen electrical power (Te)
    4: Accelerating power (Tm - Te)
    5: Bus voltage (V)
    6: dV/dt
"""

import numpy as np
from utils.pyphs_core import DynamicsCore


def ieeest_dynamics(x, ports, meta):
    """
    IEEEST PSS dynamics with multiple compensation stages.

    States: [xf1, xf2, xll1, xll2, xll3, xll4, xwo]
        xf1:  First lag filter state (A1)
        xf2:  Second lag filter state (A2)
        xll1: First lead-lag state (A3-A6, input stage)
        xll2: Second lead-lag state (A3-A6, output stage)
        xll3: Third lead-lag state (T1/T2)
        xll4: Fourth lead-lag state (T3/T4)
        xwo:  Washout filter state (T5/T6)

    Args:
        x: numpy array of 7 states
        ports: dict with input signals based on MODE
        meta: dict of PSS parameters

    Returns:
        x_dot: numpy array of state derivatives
    """
    # Extract states
    xf1, xf2, xll1, xll2, xll3, xll4, xwo = x

    # Extract parameters
    MODE = int(meta['MODE'])
    A1 = meta['A1']
    A2 = meta['A2']
    A3 = meta['A3']
    A4 = meta['A4']
    A5 = meta['A5']
    A6 = meta['A6']
    T1 = meta['T1']
    T2 = meta['T2']
    T3 = meta['T3']
    T4 = meta['T4']
    T5 = meta['T5']
    T6 = meta['T6']
    KS = meta['KS']
    LSMAX = meta['LSMAX']
    LSMIN = meta['LSMIN']

    # Get input signal based on MODE
    omega = ports.get('omega', 1.0)
    f = ports.get('f', 1.0)  # Frequency from BusFreq
    Te = ports.get('Te', 0.0)  # Electrical torque
    Tm = ports.get('Tm', 0.0)  # Mechanical torque
    V = ports.get('V', 1.0)  # Bus voltage
    dVdt = ports.get('dVdt', 0.0)  # Voltage derivative

    if MODE == 1:
        sig_in = omega - 1.0  # Speed deviation
    elif MODE == 2:
        sig_in = f - 1.0  # Frequency deviation
    elif MODE == 3:
        sig_in = Te  # Electrical power
    elif MODE == 4:
        sig_in = Tm - Te  # Accelerating power
    elif MODE == 5:
        sig_in = V  # Voltage
    elif MODE == 6:
        sig_in = dVdt  # Voltage derivative
    else:
        sig_in = 0.0  # Disabled

    # Initialize derivatives
    x_dot = np.zeros(7)

    # 1. First lag filter: d(xf1)/dt = (sig_in - xf1) / A1
    if A1 > 1e-6:
        x_dot[0] = (sig_in - xf1) / A1
        f1_out = xf1
    else:
        x_dot[0] = 0.0
        f1_out = sig_in

    # 2. Second lag filter: d(xf2)/dt = (f1_out - xf2) / A2
    if A2 > 1e-6:
        x_dot[1] = (f1_out - xf2) / A2
        f2_out = xf2
    else:
        x_dot[1] = 0.0
        f2_out = f1_out

    # 3-4. Second-order lead-lag (A3/A4, A5/A6)
    # First stage: d(xll1)/dt = (f2_out - xll1) / A4
    if A4 > 1e-6:
        x_dot[2] = (f2_out - xll1) / A4
        ll1_out = f2_out + (A3 - A4) * xll1 / A4
    else:
        x_dot[2] = 0.0
        ll1_out = A3 * f2_out

    # Second stage: d(xll2)/dt = (ll1_out - xll2) / A6
    if A6 > 1e-6:
        x_dot[3] = (ll1_out - xll2) / A6
        ll2_out = ll1_out + (A5 - A6) * xll2 / A6
    else:
        x_dot[3] = 0.0
        ll2_out = A5 * ll1_out

    # 5. Lead-lag 1: d(xll3)/dt = (ll2_out - xll3) / T2
    if T2 > 1e-6:
        x_dot[4] = (ll2_out - xll3) / T2
        ll3_out = ll2_out + (T1 - T2) * xll3 / T2
    else:
        x_dot[4] = 0.0
        ll3_out = T1 * ll2_out

    # 6. Lead-lag 2: d(xll4)/dt = (ll3_out - xll4) / T4
    if T4 > 1e-6:
        x_dot[5] = (ll3_out - xll4) / T4
        ll4_out = ll3_out + (T3 - T4) * xll4 / T4
    else:
        x_dot[5] = 0.0
        ll4_out = T3 * ll3_out

    # 7. Gain
    vks = KS * ll4_out

    # 8. Washout: d(xwo)/dt = (vks - xwo) / T6
    # Output: wo_out = T5/T6 * (vks - xwo)
    if T6 > 1e-6:
        x_dot[6] = (vks - xwo) / T6
        wo_out = T5 * (vks - xwo) / T6
    else:
        x_dot[6] = 0.0
        wo_out = T5 * vks

    return x_dot


def ieeest_output(x, ports, meta):
    """
    Compute PSS stabilizing signal output Vss.

    Args:
        x: numpy array of states
        ports: dict with input signals
        meta: dict of parameters

    Returns:
        Vss: Stabilizing signal output (limited)
    """
    xf1, xf2, xll1, xll2, xll3, xll4, xwo = x

    # Recompute washout output
    T5 = meta['T5']
    T6 = meta['T6']
    KS = meta['KS']
    LSMAX = meta['LSMAX']
    LSMIN = meta['LSMIN']
    T1 = meta['T1']
    T2 = meta['T2']
    T3 = meta['T3']
    T4 = meta['T4']
    A3 = meta['A3']
    A4 = meta['A4']
    A5 = meta['A5']
    A6 = meta['A6']
    MODE = int(meta['MODE'])

    # Reconstruct signal path for output
    # Get input signal
    omega = ports.get('omega', 1.0)
    f = ports.get('f', 1.0)
    Te = ports.get('Te', 0.0)
    Tm = ports.get('Tm', 0.0)
    V = ports.get('V', 1.0)
    dVdt = ports.get('dVdt', 0.0)

    if MODE == 1:
        sig_in = omega - 1.0
    elif MODE == 2:
        sig_in = f - 1.0
    elif MODE == 3:
        sig_in = Te
    elif MODE == 4:
        sig_in = Tm - Te
    elif MODE == 5:
        sig_in = V
    elif MODE == 6:
        sig_in = dVdt
    else:
        sig_in = 0.0

    # Process through stages
    f1_out = xf1
    f2_out = xf2
    
    if A4 > 1e-6:
        ll1_out = sig_in + (A3 - A4) * xll1 / A4 if xf2 > 0 else 0.0
    else:
        ll1_out = A3 * f2_out
    
    if A6 > 1e-6:
        ll2_out = ll1_out + (A5 - A6) * xll2 / A6
    else:
        ll2_out = A5 * ll1_out

    if T2 > 1e-6:
        ll3_out = ll2_out + (T1 - T2) * xll3 / T2
    else:
        ll3_out = T1 * ll2_out

    if T4 > 1e-6:
        ll4_out = ll3_out + (T3 - T4) * xll4 / T4
    else:
        ll4_out = T3 * ll3_out

    vks = KS * ll4_out

    if T6 > 1e-6:
        wo_out = T5 * (vks - xwo) / T6
    else:
        wo_out = T5 * vks

    # Apply output limits
    if wo_out > LSMAX:
        Vss = LSMAX
    elif wo_out < LSMIN:
        Vss = LSMIN
    else:
        Vss = wo_out

    # Check voltage enabling limits (VCL to VCU)
    VCL = meta.get('VCL', -999)
    VCU = meta.get('VCU', 999)
    V = ports.get('V', 1.0)
    
    if V < VCL or V > VCU:
        Vss = 0.0  # Disable PSS outside voltage range

    return Vss


def build_ieeest_core(pss_data, initial_conditions=None):
    """
    Build IEEEST PSS as DynamicsCore.

    Args:
        pss_data: dict with PSS parameters from JSON
        initial_conditions: dict with initial state values (optional)

    Returns:
        DynamicsCore object configured for IEEEST
    """
    # Extract and validate parameters
    MODE = int(pss_data.get('MODE', 1))
    A1 = max(pss_data.get('A1', 1.0), 0.001)
    A2 = max(pss_data.get('A2', 1.0), 0.001)
    A3 = pss_data.get('A3', 1.0)
    A4 = max(pss_data.get('A4', 1.0), 0.001)
    A5 = pss_data.get('A5', 1.0)
    A6 = max(pss_data.get('A6', 1.0), 0.001)
    T1 = pss_data.get('T1', 0.0)
    T2 = max(pss_data.get('T2', 0.0), 0.001)
    T3 = pss_data.get('T3', 0.0)
    T4 = max(pss_data.get('T4', 0.75), 0.001)
    T5 = pss_data.get('T5', 1.0)
    T6 = max(pss_data.get('T6', 4.2), 0.001)
    KS = pss_data.get('KS', -2.0)
    LSMAX = pss_data.get('LSMAX', 0.1)
    LSMIN = pss_data.get('LSMIN', -0.1)
    VCL = pss_data.get('VCL', -999.0)
    VCU = pss_data.get('VCU', 999.0)

    # Build metadata dictionary
    metadata = {
        'idx': pss_data.get('idx', 'IEEEST_1'),
        'name': pss_data.get('name', 'IEEEST_1'),
        'avr': pss_data.get('avr', None),
        'MODE': MODE,
        'A1': A1,
        'A2': A2,
        'A3': A3,
        'A4': A4,
        'A5': A5,
        'A6': A6,
        'T1': T1,
        'T2': T2,
        'T3': T3,
        'T4': T4,
        'T5': T5,
        'T6': T6,
        'KS': KS,
        'LSMAX': LSMAX,
        'LSMIN': LSMIN,
        'VCL': VCL,
        'VCU': VCU
    }

    # Initialize states: [xf1, xf2, xll1, xll2, xll3, xll4, xwo]
    if initial_conditions is not None:
        x0 = np.array([
            initial_conditions.get('xf1', 0.0),
            initial_conditions.get('xf2', 0.0),
            initial_conditions.get('xll1', 0.0),
            initial_conditions.get('xll2', 0.0),
            initial_conditions.get('xll3', 0.0),
            initial_conditions.get('xll4', 0.0),
            initial_conditions.get('xwo', 0.0)
        ])
    else:
        # Default initialization (all zeros at equilibrium)
        x0 = np.zeros(7)

    # Create DynamicsCore
    core = DynamicsCore(
        label=f"IEEEST_{metadata['idx']}",
        dynamics_fn=ieeest_dynamics
    )

    # Set metadata and component attributes
    core.set_metadata(metadata)
    core.n_states = 7
    core.output_fn = ieeest_output
    core.component_type = "pss"
    core.model_name = "IEEEST"

    return core, metadata


# Example usage
if __name__ == "__main__":
    test_params = {
        'idx': 'IEEEST_test',
        'name': 'IEEEST_test',
        'avr': 'ESST3A_1',
        'MODE': 3.0,  # Frequency deviation
        'A1': 0.0,
        'A2': 0.0,
        'A3': 0.0,
        'A4': 0.0,
        'A5': 0.0,
        'A6': 0.0,
        'T1': 0.0,
        'T2': 0.0,
        'T3': 0.0,
        'T4': 0.75,
        'T5': 1.0,
        'T6': 4.2,
        'KS': -2.0,
        'LSMAX': 0.1,
        'LSMIN': -0.1
    }

    pss = build_ieeest_core(test_params)
    test_ports = {'omega': 1.001, 'f': 1.001, 'Te': 0.8, 'Tm': 0.8, 'V': 1.0, 'dVdt': 0.0}
    x_dot = pss.dynamics_fn(pss.states, test_ports, pss.meta)
    Vss = pss.output_fn(pss.states, test_ports, pss.meta)

    print("IEEEST PSS Test")
    print(f"States: {pss.states}")
    print(f"Derivatives: {x_dot}")
    print(f"Output Vss: {Vss}")
