"""
REECA1 Renewable Energy Electrical Control Model - Port-Hamiltonian Formulation

Electrical control model that generates current commands (Ipcmd, Iqcmd)
for the converter (REGCA1). Implements reactive power/voltage control and
active power control with current limiting.

PH Structure:
    States: x = [s0_y, s1_y, piq_xi, s5_y]
        s0_y: filtered terminal voltage
        s1_y: filtered active power (Pe)
        piq_xi: reactive power PI controller integrator
        s5_y: active power order (Pord) after lag

    Hamiltonian: H = (Trv/2)*s0^2 + (Tp/2)*s1^2 + (1/2)*piq^2 + (Tpord/2)*s5^2

    Ports:
        Inputs: V, Pe, Qe (from REGCA1), Pext, Qext (from REPCA1), wg (from WTDTA1)
        Outputs: Ipcmd, Iqcmd (to REGCA1)

Reference: WECC REECA1 / REEC_A model specification
"""
from utils.pyphs_core import DynamicsCore
import numpy as np


def reeca1_dynamics(x, ports, meta):
    """
    Numerical dynamics for REECA1 electrical control.

    Args:
        x: numpy array of 4 states [s0_y, s1_y, piq_xi, s5_y]
        ports: dict with keys {'V', 'Pe', 'Qe', 'Pext', 'Qext', 'wg'}
        meta: dict of control parameters

    Returns:
        x_dot: numpy array of 4 state derivatives
    """
    s0_y, s1_y, piq_xi, s5_y = x

    V = ports.get('V', 1.0)
    Pe = ports.get('Pe', meta.get('p0', 0.0))
    Qe = ports.get('Qe', meta.get('q0', 0.0))
    Pext = ports.get('Pext', 0.0)
    Qext = ports.get('Qext', 0.0)
    wg = ports.get('wg', 1.0)

    Trv = meta['Trv']
    Tp = meta['Tp']
    Tpord = meta['Tpord']
    Kqp = meta['Kqp']
    Kqi = meta['Kqi']
    Kvp = meta['Kvp']
    Kvi = meta['Kvi']
    Vref0 = meta['Vref0']
    Vref1 = meta['Vref1']
    dbd1 = meta['dbd1']
    dbd2 = meta['dbd2']
    Kqv = meta['Kqv']
    Vdip = meta['Vdip']
    Vup = meta['Vup']
    QMax = meta['QMax']
    QMin = meta['QMin']
    VMAX = meta['VMAX']
    VMIN = meta['VMIN']
    PMAX = meta['PMAX']
    PMIN = meta['PMIN']
    Imax = meta['Imax']
    dPmax = meta['dPmax']
    dPmin = meta['dPmin']
    PFFLAG = meta['PFFLAG']
    VFLAG = meta['VFLAG']
    QFLAG = meta['QFLAG']
    PFLAG = meta['PFLAG']
    PQFLAG = meta['PQFLAG']

    Iqcmd0 = meta.get('Iqcmd0', 0.0)
    Ipcmd0 = meta.get('Ipcmd0', 0.0)
    qref0 = meta.get('qref0', 0.0)

    # Voltage capping
    vp = max(V, 0.01)

    # --- Voltage dip detection ---
    Volt_dip = 1.0 if (V < Vdip or V > Vup) else 0.0

    # --- s0: Voltage filter ---
    ds0 = (V - s0_y) / Trv

    # --- s1: Pe filter ---
    ds1 = (Pe - s1_y) / Tp

    # --- Reactive power control path ---
    # Qref (external Q reference, from REPCA1 or default)
    Qref = -qref0 + Qext

    # Power factor control (PFFLAG=1) or Q control (PFFLAG=0)
    if PFFLAG > 0.5:
        pfaref0 = meta.get('pfaref0', 0.0)
        Qcpf = s1_y * np.tan(pfaref0) if abs(meta.get('p0', 0.0)) > 1e-6 else meta.get('q0', 0.0)
        PFsel = Qcpf
    else:
        PFsel = Qref

    # Clamp PFsel to Q limits
    PFsel = np.clip(PFsel, QMin, QMax)

    # Reactive power error
    Qerr = PFsel - Qe

    # --- PI for Q (piq_xi) ---
    # VFLAG=1: Q control drives PIQ, output is voltage ref
    # VFLAG=0: direct voltage ref (Vref1)
    if VFLAG > 0.5:
        piq_input = Qerr
    else:
        piq_input = 0.0

    # PI integrator with anti-windup
    piq_p = Kqp * piq_input
    piq_y = piq_p + piq_xi

    # Clamp PI output
    piq_y_clamped = np.clip(piq_y, VMIN, VMAX)

    # Anti-windup: freeze integrator if at limits
    if (piq_y >= VMAX and Kqi * piq_input > 0) or \
       (piq_y <= VMIN and Kqi * piq_input < 0):
        dpiq = 0.0
    elif Volt_dip > 0.5:
        dpiq = 0.0  # Freeze during voltage dip
    else:
        dpiq = Kqi * piq_input

    # --- Voltage selection ---
    if VFLAG > 0.5:
        Vsel = piq_y_clamped
    else:
        Vsel = Vref1 + PFsel
    Vsel = np.clip(Vsel, VMIN, VMAX)

    # --- Voltage PI for Iq (when QFLAG=1) ---
    if QFLAG > 0.5:
        Verr_piv = Vsel - s0_y
        # Use proportional only for simplified model
        PIV_y = Kvp * Verr_piv + Kvi * piq_xi  # Reuse integrator state
        Qsel = PIV_y
    else:
        # Direct Iq from filtered Q
        Qsel = Iqcmd0  # Simplified: use initial Iq

    # --- Upper portion: Iqinj (voltage-dependent injection) ---
    Verr = Vref0 - s0_y
    # Deadband
    if Verr < dbd1:
        dbV_y = Verr - dbd1
    elif Verr > dbd2:
        dbV_y = Verr - dbd2
    else:
        dbV_y = 0.0

    Iqinj = dbV_y * Kqv * Volt_dip

    # --- Iqcmd: Qsel + Iqinj with current limits ---
    Iqcmd_raw = Qsel + Iqinj

    # Current limit calculation
    if PQFLAG > 0.5:
        # P priority: Ipmax first, then Iqmax from remaining
        Ipmax = Imax
        Iqmax_sq = max(0.0, Imax**2 - Ipcmd0**2)
        Iqmax = np.sqrt(Iqmax_sq)
    else:
        # Q priority: Iqmax first, then Ipmax from remaining
        Iqmax = Imax
        Ipmax_sq = max(0.0, Imax**2 - Iqcmd0**2)
        Ipmax = np.sqrt(Ipmax_sq)

    Iqcmd = np.clip(Iqcmd_raw, -Iqmax, Iqmax)

    # --- Active power path ---
    # Power reference
    Pref = meta.get('p0', 0.0) / max(wg, 0.01) + Pext

    # Rate limit on Pref
    # (simplified: Pref filtered through s5 lag)

    # Power selection based on PFLAG
    if PFLAG > 0.5:
        Psel = wg * Pref
    else:
        Psel = Pref

    # --- s5: Pord lag with limits ---
    s5_target = np.clip(Psel, PMIN, PMAX)
    ds5 = (s5_target - s5_y) / Tpord

    # Freeze during voltage dip
    if Volt_dip > 0.5:
        ds5 = 0.0

    # --- Ipcmd from Pord ---
    Ipcmd = np.clip(s5_y / vp, 0.0, Ipmax)

    x_dot = np.zeros(4)
    x_dot[0] = ds0
    x_dot[1] = ds1
    x_dot[2] = dpiq
    x_dot[3] = ds5

    return x_dot


def reeca1_output(x, ports, meta):
    """
    Compute electrical control outputs: Ipcmd, Iqcmd.

    Args:
        x: numpy array [s0_y, s1_y, piq_xi, s5_y]
        ports: dict with 'V', 'Pe', 'Qe', 'wg'
        meta: dict of parameters

    Returns:
        dict with 'Ipcmd', 'Iqcmd'
    """
    s0_y, s1_y, piq_xi, s5_y = x
    V = ports.get('V', 1.0)
    vp = max(V, 0.01)

    # Low-voltage gain (LVG) used by REGCA1; replicate here so active current tracks power target
    Lvpnt0 = meta.get('Lvpnt0', 0.4)
    Lvpnt1 = meta.get('Lvpnt1', 1.0)
    if V <= Lvpnt0:
        LVG_y = 0.0
    elif V <= Lvpnt1:
        LVG_y = (V - Lvpnt0) / (Lvpnt1 - Lvpnt0)
    else:
        LVG_y = 1.0
    LVG_y = max(LVG_y, 0.01)

    Vref0 = meta['Vref0']
    dbd1 = meta['dbd1']
    dbd2 = meta['dbd2']
    Kqv = meta['Kqv']
    Vdip = meta['Vdip']
    Vup = meta['Vup']
    Imax = meta['Imax']
    PQFLAG = meta['PQFLAG']
    QFLAG = meta['QFLAG']
    Kvp = meta['Kvp']
    Iqcmd0 = meta.get('Iqcmd0', 0.0)
    Ipcmd0 = meta.get('Ipcmd0', 0.0)

    Volt_dip = 1.0 if (V < Vdip or V > Vup) else 0.0

    # Iqinj
    Verr = Vref0 - s0_y
    if Verr < dbd1:
        dbV_y = Verr - dbd1
    elif Verr > dbd2:
        dbV_y = Verr - dbd2
    else:
        dbV_y = 0.0
    Iqinj = dbV_y * Kqv * Volt_dip

    # Qsel
    if QFLAG > 0.5:
        Qsel = Kvp * (meta.get('Vref1', 1.0) - s0_y) + piq_xi
    else:
        Qsel = Iqcmd0

    # Current limits
    if PQFLAG > 0.5:
        Ipmax = Imax
        Iqmax = np.sqrt(max(0.0, Imax**2 - (s5_y / vp)**2))
    else:
        Iqmax = Imax
        Ipmax = np.sqrt(max(0.0, Imax**2 - (Qsel + Iqinj)**2))

    Iqcmd = np.clip(Qsel + Iqinj, -Iqmax, Iqmax)
    # Scale by LVG so Ip = Pord / (V * LVG), keeping Pe ≈ Pord even when LVG < 1
    Ipcmd = np.clip(s5_y / (vp * LVG_y), 0.0, Ipmax)

    return {'Ipcmd': Ipcmd, 'Iqcmd': Iqcmd}


def build_reeca1_core(ree_data, S_system=100.0):
    """Build REECA1 electrical control as DynamicsCore.

    Args:
        ree_data: dict with control parameters from JSON
        S_system: system base power (MVA)

    Returns:
        core: DynamicsCore object
        metadata: dict with additional info
    """
    core = DynamicsCore(label=f'REECA1_{ree_data["idx"]}', dynamics_fn=reeca1_dynamics)

    metadata = {
        'idx': ree_data['idx'],
        'reg': ree_data['reg'],
        'PFFLAG': ree_data.get('PFFLAG', 0),
        'VFLAG': ree_data.get('VFLAG', 0),
        'QFLAG': ree_data.get('QFLAG', 0),
        'PFLAG': ree_data.get('PFLAG', 0),
        'PQFLAG': ree_data.get('PQFLAG', 0),
        'Vdip': ree_data.get('Vdip', 0.8),
        'Vup': ree_data.get('Vup', 1.1),
        'Trv': max(ree_data.get('Trv', 0.02), 0.001),
        'dbd1': ree_data.get('dbd1', -0.02),
        'dbd2': ree_data.get('dbd2', 0.02),
        'Kqv': ree_data.get('Kqv', 20.0),
        'Iqh1': ree_data.get('Iqh1', 999.0),
        'Iql1': ree_data.get('Iql1', -999.0),
        'Vref0': ree_data.get('Vref0', 1.0),
        'Iqfrz': ree_data.get('Iqfrz', 0.0),
        'Thld': ree_data.get('Thld', 0.0),
        'Thld2': ree_data.get('Thld2', 0.0),
        'Tp': max(ree_data.get('Tp', 0.02), 0.001),
        'QMax': ree_data.get('QMax', 999.0),
        'QMin': ree_data.get('QMin', -999.0),
        'VMAX': ree_data.get('VMAX', 999.0),
        'VMIN': ree_data.get('VMIN', -999.0),
        'Kqp': ree_data.get('Kqp', 1.0),
        'Kqi': ree_data.get('Kqi', 0.1),
        'Kvp': ree_data.get('Kvp', 1.0),
        'Kvi': ree_data.get('Kvi', 0.1),
        'Vref1': ree_data.get('Vref1', 1.0),
        'Tiq': max(ree_data.get('Tiq', 0.02), 0.001),
        'dPmax': ree_data.get('dPmax', 999.0),
        'dPmin': ree_data.get('dPmin', -999.0),
        'PMAX': ree_data.get('PMAX', 999.0),
        'PMIN': ree_data.get('PMIN', -999.0),
        'Imax': ree_data.get('Imax', 10.0),
        'Tpord': max(ree_data.get('Tpord', 0.02), 0.001),
        'p0': 0.0,  # Set during initialization
        'q0': 0.0,
        'Ipcmd0': 0.0,
        'Iqcmd0': 0.0,
        'qref0': 0.0,
        'pfaref0': 0.0,
    }

    # Symbolic PH structure
    s0, s1, piq, s5 = core.symbols(['s0_vf', 's1_pf', 'piq_xi', 's5_pord'])
    Trv_sym, Tp_sym, Tpord_sym = core.symbols(['Trv', 'Tp', 'Tpord'])

    H_ctrl = (Trv_sym / 2) * s0**2 + (Tp_sym / 2) * s1**2 + \
             piq**2 / 2 + (Tpord_sym / 2) * s5**2
    core.add_storages([s0, s1, piq, s5], H_ctrl)

    core.subs.update({
        Trv_sym: metadata['Trv'],
        Tp_sym: metadata['Tp'],
        Tpord_sym: metadata['Tpord'],
    })

    core.set_metadata(metadata)
    core.n_states = 4
    core.output_fn = reeca1_output
    core.component_type = "renewable"
    core.model_name = "REECA1"

    def init_fn(P0=0.35, Q0=0.1, V0=1.0, **kwargs):
        """Initialize electrical control at equilibrium."""
        Ipcmd0 = P0 / V0 if V0 > 0.01 else 0.0
        # Sign convention: positive reactive power corresponds to positive Iq
        # (REECA1 outputs Iqcmd that should drive REGCA1 Qe = Iqout * V).
        Iqcmd0 = Q0 / V0 if V0 > 0.01 else 0.0
        metadata['p0'] = P0
        metadata['q0'] = Q0
        metadata['Ipcmd0'] = Ipcmd0
        metadata['Iqcmd0'] = Iqcmd0
        metadata['pfaref0'] = np.arctan2(Q0, P0) if abs(P0) > 1e-6 else 0.0

        # qref0: depends on QFLAG and VFLAG
        if metadata['QFLAG'] > 0.5:
            metadata['qref0'] = V0 - metadata['Vref1']
        else:
            # Qref = -qref0 + Qext. With Qext=0, want Qref=Q0 at equilibrium.
            metadata['qref0'] = -Iqcmd0 * V0

        # Initial states: all at equilibrium
        s0_0 = V0      # Voltage filter output = V
        s1_0 = P0      # Power filter output = Pe
        piq_0 = 0.0    # PI integrator = 0 at equilibrium
        s5_0 = P0      # Pord = P0 at equilibrium
        return np.array([s0_0, s1_0, piq_0, s5_0])

    core.init_fn = init_fn

    return core, metadata
