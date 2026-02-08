"""ST2CUT PSS Model - Dual-Input Power System Stabilizer

Port-Hamiltonian implementation of ST2CUT PSS with dual input capability.
Based on IEEE/CIGRE PSS models.

Signal path:
    Input 1 (MODE) -> [K1/(1+sT1) transducer] -> sig1
    Input 2 (MODE2) -> [K2/(1+sT2) transducer] -> sig2
    Sum: sig1 + sig2
    -> [sT3/(1+sT4) washout
]
    -> [lead-lag (1+sT5)/(1+sT6)]
    -> [lead-lag (1+sT7)/(1+sT8)]
    -> [lead-lag (1+sT9)/(1+sT10)]
    -> [LSMAX/LSMIN limits]
    -> Vss (stabilizing signal to exciter)

Input modes (MODE, MODE2):
    0: Disabled
    1: Rotor speed deviation (ω - 1)
    2: Bus frequency deviation (f - 1)
    3: Gen electrical power (Te)
    4: Accelerating power (Tm - Te)
    5: Bus voltage (V)
    6: dV/dt
"""

import numpy as np
from utils.pyphs_core import DynamicsCore


def st2cut_dynamics(x, ports, meta):
    """
    ST2CUT PSS dynamics with dual input and multiple lead-lag stages.

    States: [xl1, xl2, xwo, xll1, xll2, xll3]
        xl1:  Transducer 1 lag state (T1)
        xl2:  Transducer 2 lag state (T2)
        xwo:  Washout filter state (T3/T4)
        xll1: Lead-lag 1 state (T5/T6)
        xll2: Lead-lag 2 state (T7/T8)
        xll3: Lead-lag 3 state (T9/T10)

    Args:
        x: numpy array of 6 states
        ports: dict with input signals based on MODE/MODE2
        meta: dict of PSS parameters

    Returns:
        x_dot: numpy array of state derivatives
    """
    # Extract states
    xl1, xl2, xwo, xll1, xll2, xll3 = x

    # Extract parameters
    MODE = int(meta.get('MODE', 1))
    MODE2 = int(meta.get('MODE2', 0))
    K1 = meta['K1']
    K2 = meta['K2']
    T1 = meta['T1']
    T2 = meta['T2']
    T3 = meta['T3']
    T4 = meta['T4']
    T5 = meta['T5']
    T6 = meta['T6']
    T7 = meta['T7']
    T8 = meta['T8']
    T9 = meta['T9']
    T10 = meta['T10']

    # Get input signals
    omega = ports.get('omega', 1.0)
    f = ports.get('f', 1.0)  # Frequency from BusFreq
    f2 = ports.get('f2', 1.0)  # Secondary frequency
    Te = ports.get('Te', 0.0)  # Electrical torque
    Tm = ports.get('Tm', 0.0)  # Mechanical torque
    V = ports.get('V', 1.0)  # Bus voltage
    V2 = ports.get('V2', 1.0)  # Secondary bus voltage
    dVdt = ports.get('dVdt', 0.0)  # Voltage derivative
    dV2dt = ports.get('dV2dt', 0.0)  # Secondary voltage derivative

    # Select input signal 1 based on MODE
    if MODE == 1:
        sig1_in = omega - 1.0  # Speed deviation
    elif MODE == 2:
        sig1_in = f - 1.0  # Frequency deviation
    elif MODE == 3:
        sig1_in = Te  # Electrical power
    elif MODE == 4:
        sig1_in = Tm - Te  # Accelerating power
    elif MODE == 5:
        sig1_in = V  # Voltage
    elif MODE == 6:
        sig1_in = dVdt  # Voltage derivative
    else:
        sig1_in = 0.0  # Disabled

    # Select input signal 2 based on MODE2
    if MODE2 == 1:
        sig2_in = omega - 1.0
    elif MODE2 == 2:
        sig2_in = f2 - 1.0
    elif MODE2 == 3:
        sig2_in = Te
    elif MODE2 == 4:
        sig2_in = Tm - Te
    elif MODE2 == 5:
        sig2_in = V2
    elif MODE2 == 6:
        sig2_in = dV2dt
    else:
        sig2_in = 0.0  # Disabled

    # Initialize derivatives
    x_dot = np.zeros(6)

    # 1. Transducer 1 lag: d(xl1)/dt = (sig1_in - xl1) / T1
    if T1 > 1e-6:
        x_dot[0] = (sig1_in - xl1) / T1
        L1_out = K1 * xl1
    else:
        x_dot[0] = 0.0
        L1_out = K1 * sig1_in

    # 2. Transducer 2 lag: d(xl2)/dt = (sig2_in - xl2) / T2
    if T2 > 1e-6:
        x_dot[1] = (sig2_in - xl2) / T2
        L2_out = K2 * xl2
    else:
        x_dot[1] = 0.0
        L2_out = K2 * sig2_in

    # 3. Sum of inputs
    IN = L1_out + L2_out

    # 4. Washout: d(xwo)/dt = (IN - xwo) / T4
    # Output: wo_out = T3/T4 * (IN - xwo)
    if T4 > 1e-6:
        x_dot[2] = (IN - xwo) / T4
        wo_out = T3 * (IN - xwo) / T4
    else:
        x_dot[2] = 0.0
        wo_out = T3 * IN

    # 5. Lead-lag 1: d(xll1)/dt = (wo_out - xll1) / T6
    if T6 > 1e-6:
        x_dot[3] = (wo_out - xll1) / T6
        ll1_out = wo_out + (T5 - T6) * xll1 / T6
    else:
        x_dot[3] = 0.0
        ll1_out = T5 * wo_out

    # 6. Lead-lag 2: d(xll2)/dt = (ll1_out - xll2) / T8
    if T8 > 1e-6:
        x_dot[4] = (ll1_out - xll2) / T8
        ll2_out = ll1_out + (T7 - T8) * xll2 / T8
    else:
        x_dot[4] = 0.0
        ll2_out = T7 * ll1_out

    # 7. Lead-lag 3: d(xll3)/dt = (ll2_out - xll3) / T10
    if T10 > 1e-6:
        x_dot[5] = (ll2_out - xll3) / T10
        # ll3_out computed in output function
    else:
        x_dot[5] = 0.0

    return x_dot


def st2cut_output(x, ports, meta):
    """
    Compute PSS stabilizing signal output Vss.

    Args:
        x: numpy array of states
        ports: dict with input signals
        meta: dict of parameters

    Returns:
        Vss: Stabilizing signal output (limited)
    """
    xl1, xl2, xwo, xll1, xll2, xll3 = x

    # Extract parameters
    K1 = meta['K1']
    K2 = meta['K2']
    T3 = meta['T3']
    T4 = meta['T4']
    T5 = meta['T5']
    T6 = meta['T6']
    T7 = meta['T7']
    T8 = meta['T8']
    T9 = meta['T9']
    T10 = meta['T10']
    LSMAX = meta['LSMAX']
    LSMIN = meta['LSMIN']
    MODE = int(meta.get('MODE', 1))
    MODE2 = int(meta.get('MODE2', 0))

    # Recompute signal path for output
    omega = ports.get('omega', 1.0)
    f = ports.get('f', 1.0)
    f2 = ports.get('f2', 1.0)
    Te = ports.get('Te', 0.0)
    Tm = ports.get('Tm', 0.0)
    V = ports.get('V', 1.0)
    V2 = ports.get('V2', 1.0)
    dVdt = ports.get('dVdt', 0.0)
    dV2dt = ports.get('dV2dt', 0.0)

    # Select input signals
    if MODE == 1:
        sig1_in = omega - 1.0
    elif MODE == 2:
        sig1_in = f - 1.0
    elif MODE == 3:
        sig1_in = Te
    elif MODE == 4:
        sig1_in = Tm - Te
    elif MODE == 5:
        sig1_in = V
    elif MODE == 6:
        sig1_in = dVdt
    else:
        sig1_in = 0.0

    if MODE2 == 1:
        sig2_in = omega - 1.0
    elif MODE2 == 2:
        sig2_in = f2 - 1.0
    elif MODE2 == 3:
        sig2_in = Te
    elif MODE2 == 4:
        sig2_in = Tm - Te
    elif MODE2 == 5:
        sig2_in = V2
    elif MODE2 == 6:
        sig2_in = dV2dt
    else:
        sig2_in = 0.0

    # Process through stages
    L1_out = K1 * xl1
    L2_out = K2 * xl2
    IN = L1_out + L2_out

    if T4 > 1e-6:
        wo_out = T3 * (IN - xwo) / T4
    else:
        wo_out = T3 * IN

    if T6 > 1e-6:
        ll1_out = wo_out + (T5 - T6) * xll1 / T6
    else:
        ll1_out = T5 * wo_out

    if T8 > 1e-6:
        ll2_out = ll1_out + (T7 - T8) * xll2 / T8
    else:
        ll2_out = T7 * ll1_out

    if T10 > 1e-6:
        ll3_out = ll2_out + (T9 - T10) * xll3 / T10
    else:
        ll3_out = T9 * ll2_out

    # Apply output limits
    if ll3_out > LSMAX:
        Vss = LSMAX
    elif ll3_out < LSMIN:
        Vss = LSMIN
    else:
        Vss = ll3_out

    # Check voltage enabling limits
    VCL = meta.get('VCL', -999)
    VCU = meta.get('VCU', 999)
    V_enable = ports.get('V', 1.0)
    
    if V_enable < VCL or V_enable > VCU:
        Vss = 0.0  # Disable PSS outside voltage range

    return Vss


def build_st2cut_core(pss_data, initial_conditions=None):
    """
    Build ST2CUT PSS as DynamicsCore.

    Args:
        pss_data: dict with PSS parameters from JSON
        initial_conditions: dict with initial state values (optional)

    Returns:
        DynamicsCore object configured for ST2CUT
    """
    # Extract and validate parameters
    MODE = int(pss_data.get('MODE', 1))
    MODE2 = int(pss_data.get('MODE2', 0))
    K1 = pss_data.get('K1', 1.0)
    K2 = pss_data.get('K2', 0.0)
    T1 = max(pss_data.get('T1', 0.0), 0.001)
    T2 = max(pss_data.get('T2', 0.0), 0.001)
    T3 = pss_data.get('T3', 30.0)
    T4 = max(pss_data.get('T4', 30.0), 0.001)
    T5 = pss_data.get('T5', 0.23)
    T6 = max(pss_data.get('T6', 0.025), 0.001)
    T7 = pss_data.get('T7', 0.23)
    T8 = max(pss_data.get('T8', 0.025), 0.001)
    T9 = pss_data.get('T9', 0.0)
    T10 = max(pss_data.get('T10', 0.001), 0.001)
    LSMAX = pss_data.get('LSMAX', 0.06)
    LSMIN = pss_data.get('LSMIN', -0.06)
    VCL = pss_data.get('VCL', -999.0)
    VCU = pss_data.get('VCU', 999.0)

    # Build metadata dictionary
    metadata = {
        'idx': pss_data.get('idx', 'ST2CUT_1'),
        'name': pss_data.get('name', 'ST2CUT_1'),
        'avr': pss_data.get('avr', None),
        'MODE': MODE,
        'MODE2': MODE2,
        'K1': K1,
        'K2': K2,
        'T1': T1,
        'T2': T2,
        'T3': T3,
        'T4': T4,
        'T5': T5,
        'T6': T6,
        'T7': T7,
        'T8': T8,
        'T9': T9,
        'T10': T10,
        'LSMAX': LSMAX,
        'LSMIN': LSMIN,
        'VCL': VCL,
        'VCU': VCU
    }

    # Initialize states: [xl1, xl2, xwo, xll1, xll2, xll3]
    if initial_conditions is not None:
        x0 = np.array([
            initial_conditions.get('xl1', 0.0),
            initial_conditions.get('xl2', 0.0),
            initial_conditions.get('xwo', 0.0),
            initial_conditions.get('xll1', 0.0),
            initial_conditions.get('xll2', 0.0),
            initial_conditions.get('xll3', 0.0)
        ])
    else:
        # Default initialization (all zeros at equilibrium)
        x0 = np.zeros(6)

    # Create DynamicsCore
    core = DynamicsCore(
        label=f"ST2CUT_{metadata['idx']}",
        dynamics_fn=st2cut_dynamics
    )

    # Set metadata and component attributes
    core.set_metadata(metadata)
    core.n_states = 6
    core.output_fn = st2cut_output
    core.component_type = "pss"
    core.model_name = "ST2CUT"

    return core, metadata


# Example usage
if __name__ == "__main__":
    test_params = {
        'idx': 'ST2CUT_test',
        'name': 'ST2CUT_test',
        'avr': 'ESST3A_1',
        'MODE': 1.0,  # Speed deviation
        'MODE2': 0.0,  # Disabled
        'K1': 10.0,
        'K2': 0.0,
        'T1': 0.0,
        'T2': 0.0,
        'T3': 3.0,
        'T4': 3.0,
        'T5': 0.15,
        'T6': 0.05,
        'T7': 0.15,
        'T8': 0.05,
        'T9': 0.15,
        'T10': 0.05,
        'LSMAX': 0.05,
        'LSMIN': -0.05
    }

    pss = build_st2cut_core(test_params)
    test_ports = {'omega': 1.001, 'f': 1.0, 'Te': 0.8, 'Tm': 0.8, 'V': 1.0, 'dVdt': 0.0}
    x_dot = pss.dynamics_fn(pss.states, test_ports, pss.meta)
    Vss = pss.output_fn(pss.states, test_ports, pss.meta)

    print("ST2CUT PSS Test")
    print(f"States: {pss.states}")
    print(f"Derivatives: {x_dot}")
    print(f"Output Vss: {Vss}")
