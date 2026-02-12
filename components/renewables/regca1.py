"""
REGCA1 Renewable Energy Generator/Converter Model - Port-Hamiltonian Formulation

Converter model that interfaces with the network. Converts current commands
(Ipcmd, Iqcmd) to actual current injections (Ip, Iq) via lag blocks with
rate limiting and voltage-dependent current management.

PH Structure:
    States: x = [s0_y, s1_y, s2_y]
        s0_y: active current output (Ip) after lag and rate limits
        s1_y: reactive current output (Iq) after lag
        s2_y: filtered voltage for LVPL

    Hamiltonian: H = (Tg/2)*s0^2 + (Tg/2)*s1^2 + (Tfltr/2)*s2^2

    Ports:
        Inputs: Ipcmd, Iqcmd (from REECA1), V (bus voltage)
        Outputs: Pe, Qe (active/reactive power to network)

Reference: WECC REGCA1 / REGC_A model specification
"""
from utils.pyphs_core import DynamicsCore
import numpy as np


def regca1_dynamics(x, ports, meta):
    """
    Numerical dynamics for REGCA1 converter.

    Args:
        x: numpy array of 3 states [s0_y, s1_y, s2_y]
            s0_y: Ip output (active current, after lag/rate limit)
            s1_y: Iq output (reactive current, after lag, note: K=-1)
            s2_y: filtered voltage for LVPL
        ports: dict with keys {'Ipcmd', 'Iqcmd', 'V'}
        meta: dict of converter parameters

    Returns:
        x_dot: numpy array of 3 state derivatives
    """
    s0_y, s1_y, s2_y = x

    Ipcmd = ports.get('Ipcmd', meta.get('Ipcmd0', 0.0))
    Iqcmd = ports.get('Iqcmd', meta.get('Iqcmd0', 0.0))
    V = ports.get('V', 1.0)

    Tg = meta['Tg']
    Tfltr = meta['Tfltr']
    Rrpwr = meta['Rrpwr']
    Iqrmax = meta['Iqrmax']
    Iqrmin = meta['Iqrmin']
    Khv = meta['Khv']
    Volim = meta['Volim']
    Lvplsw = meta['Lvplsw']
    Lvpl1 = meta['Lvpl1']
    Brkpt = meta['Brkpt']
    Zerox = meta['Zerox']
    Lvpnt0 = meta['Lvpnt0']
    Lvpnt1 = meta['Lvpnt1']
    Iolim = meta['Iolim']

    # --- LVPL (Low Voltage Power Logic) ---
    # Piecewise upper limit on Ip based on filtered voltage
    if Lvplsw > 0.5:
        if s2_y <= Zerox:
            LVPL_y = 0.0
        elif s2_y <= Brkpt:
            kLVPL = Lvpl1 / (Brkpt - Zerox)
            LVPL_y = (s2_y - Zerox) * kLVPL
        else:
            LVPL_y = 9999.0
    else:
        LVPL_y = 9999.0

    # --- LVG (Low Voltage Gain) for active current management ---
    if V <= Lvpnt0:
        LVG_y = 0.0
    elif V <= Lvpnt1:
        kLVG = 1.0 / (Lvpnt1 - Lvpnt0)
        LVG_y = (V - Lvpnt0) * kLVG
    else:
        LVG_y = 1.0

    # --- S0: Lag for Ip with LVPL upper limit and rate limit ---
    s0_upper = min(LVPL_y, 9999.0)
    s0_target = Ipcmd

    # Lag dynamics: T * ds0/dt = u - y, with upper limit and rate limit
    s0_error = s0_target - s0_y
    s0_rate = s0_error / Tg

    # Apply rate limit (Rrpwr for upper rate)
    s0_rate = min(s0_rate, Rrpwr)

    # Apply anti-windup: if at upper limit and rate would push higher, clamp
    if s0_y >= s0_upper and s0_rate > 0:
        s0_rate = 0.0

    # --- S1: Lag for Iq with rate limits ---
    # Note: Andes uses K=-1 for this lag, so S1_y = -Iq_delayed
    # We track the actual Iq output directly
    s1_target = Iqcmd
    s1_error = s1_target - s1_y
    s1_rate = s1_error / Tg

    # Apply rate limits on Iq
    s1_rate = np.clip(s1_rate, Iqrmin / Tg, Iqrmax / Tg)

    # --- S2: Voltage filter for LVPL ---
    s2_rate = (V - s2_y) / Tfltr

    x_dot = np.zeros(3)
    x_dot[0] = s0_rate
    x_dot[1] = s1_rate
    x_dot[2] = s2_rate

    return x_dot


def regca1_output(x, ports, meta):
    """
    Compute converter outputs: Pe, Qe, Ipout, Iqout.

    Args:
        x: numpy array [s0_y, s1_y, s2_y]
        ports: dict with 'V' (bus voltage)
        meta: dict of parameters

    Returns:
        dict with 'Pe', 'Qe', 'Ipout', 'Iqout'
    """
    s0_y, s1_y, s2_y = x
    V = ports.get('V', 1.0)

    Lvpnt0 = meta['Lvpnt0']
    Lvpnt1 = meta['Lvpnt1']
    Khv = meta['Khv']
    Volim = meta['Volim']
    Iolim = meta['Iolim']

    # LVG: low voltage active current gain
    if V <= Lvpnt0:
        LVG_y = 0.0
    elif V <= Lvpnt1:
        kLVG = 1.0 / (Lvpnt1 - Lvpnt0)
        LVG_y = (V - Lvpnt0) * kLVG
    else:
        LVG_y = 1.0

    # Active current output
    Ipout = s0_y * LVG_y

    # High voltage reactive current management
    HVG_y = max(0.0, Khv * (V - Volim))

    # Reactive current output with lower limit
    Iqout = max(s1_y - HVG_y, Iolim)

    # Power outputs
    Pe = Ipout * V
    Qe = Iqout * V

    return {'Pe': Pe, 'Qe': Qe, 'Ipout': Ipout, 'Iqout': Iqout}


def build_regca1_core(reg_data, S_system=100.0):
    """Build REGCA1 converter as DynamicsCore.

    Args:
        reg_data: dict with converter parameters from JSON
        S_system: system base power (MVA)

    Returns:
        core: DynamicsCore object
        metadata: dict with additional info
    """
    core = DynamicsCore(label=f'REGCA1_{reg_data["idx"]}', dynamics_fn=regca1_dynamics)

    Sn = reg_data.get('Sn', 100.0)

    metadata = {
        'idx': reg_data['idx'],
        'bus': reg_data['bus'],
        'gen': reg_data['gen'],
        'Sn': Sn,
        'Tg': max(reg_data.get('Tg', 0.1), 0.001),
        'Rrpwr': reg_data.get('Rrpwr', 999.0),
        'Brkpt': reg_data.get('Brkpt', 0.8),
        'Zerox': reg_data.get('Zerox', 0.5),
        'Lvplsw': reg_data.get('Lvplsw', 1.0),
        'Lvpl1': reg_data.get('Lvpl1', 1.0),
        'Volim': reg_data.get('Volim', 1.2),
        'Lvpnt1': reg_data.get('Lvpnt1', 1.0),
        'Lvpnt0': reg_data.get('Lvpnt0', 0.4),
        'Iolim': reg_data.get('Iolim', 0.0),
        'Tfltr': max(reg_data.get('Tfltr', 0.1), 0.001),
        'Khv': reg_data.get('Khv', 0.7),
        'Iqrmax': reg_data.get('Iqrmax', 1.0),
        'Iqrmin': reg_data.get('Iqrmin', -1.0),
        'Accel': reg_data.get('Accel', 0.0),
        'gammap': reg_data.get('gammap', 1.0),
        'gammaq': reg_data.get('gammaq', 1.0),
        'Ipcmd0': 0.0,  # Set during initialization
        'Iqcmd0': 0.0,  # Set during initialization
    }

    # Symbolic PH structure
    s0, s1, s2 = core.symbols(['s0_ip', 's1_iq', 's2_vf'])
    Tg_sym = core.symbols('Tg')
    Tfltr_sym = core.symbols('Tfltr')

    H_conv = (Tg_sym / 2) * s0**2 + (Tg_sym / 2) * s1**2 + (Tfltr_sym / 2) * s2**2
    core.add_storages([s0, s1, s2], H_conv)

    # Dissipation (lag dynamics: z = w/T)
    w0, w1, w2 = core.symbols(['w_s0', 'w_s1', 'w_s2'])
    core.add_dissipations([w0, w1, w2],
                          [w0 / Tg_sym, w1 / Tg_sym, w2 / Tfltr_sym])

    # Ports
    Ipcmd_in, Iqcmd_in, V_in = core.symbols(['Ipcmd_in', 'Iqcmd_in', 'V_in'])
    Pe_out, Qe_out = core.symbols(['Pe_out', 'Qe_out'])
    core.add_ports([Ipcmd_in, Iqcmd_in, V_in], [Pe_out, Qe_out])

    core.subs.update({
        Tg_sym: metadata['Tg'],
        Tfltr_sym: metadata['Tfltr'],
    })

    core.set_metadata(metadata)
    core.n_states = 3
    core.output_fn = regca1_output
    core.component_type = "renewable"
    core.model_name = "REGCA1"

    def init_fn(P0=0.35, Q0=0.1, V0=1.0, **kwargs):
        """Initialize converter at equilibrium."""
        Ipcmd0 = P0 / V0 if V0 > 0.01 else 0.0
        # Sign convention: REGCA1 outputs Qe = Iqout * V.
        # Therefore positive reactive power setpoint should correspond to positive Iq.
        Iqcmd0 = Q0 / V0 if V0 > 0.01 else 0.0
        metadata['Ipcmd0'] = Ipcmd0
        metadata['Iqcmd0'] = Iqcmd0
        # At equilibrium: s0=Ipcmd0, s1=Iqcmd0, s2=V0
        return np.array([Ipcmd0, Iqcmd0, V0])

    core.init_fn = init_fn

    return core, metadata
