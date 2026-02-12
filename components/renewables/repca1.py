"""
REPCA1 Renewable Energy Plant Controller - Port-Hamiltonian Formulation

Plant-level controller that provides Pext and Qext signals to the
electrical control model (REECA1). Implements voltage/reactive power
regulation and frequency-based active power control.

PH Structure:
    States: x = [s0_y, s1_y, s2_xi, s4_y, s5_xi]
        s0_y: filtered voltage (or compensated voltage)
        s1_y: filtered reactive power (Qline)
        s2_xi: reactive power PI controller integrator
        s4_y: filtered active power (Pline)
        s5_xi: active power PI controller integrator

    Hamiltonian: H = (Tfltr/2)*s0^2 + (Tfltr/2)*s1^2 + (1/2)*s2^2 +
                     (Tp/2)*s4^2 + (1/2)*s5^2

    Ports:
        Inputs: V (bus voltage), f (frequency), Pline, Qline
        Outputs: Pext, Qext (to REECA1)

Reference: WECC REPCA1 model specification
"""
from utils.pyphs_core import DynamicsCore
import numpy as np


def repca1_dynamics(x, ports, meta):
    """
    Numerical dynamics for REPCA1 plant controller.

    Args:
        x: numpy array of 5 states [s0_y, s1_y, s2_xi, s4_y, s5_xi]
        ports: dict with keys {'V', 'f', 'Pline', 'Qline', 'a'}
        meta: dict of plant controller parameters

    Returns:
        x_dot: numpy array of 5 state derivatives
    """
    s0_y, s1_y, s2_xi, s4_y, s5_xi = x

    V = ports.get('V', 1.0)
    f = ports.get('f', 1.0)
    Pline = ports.get('Pline', meta.get('Pline0', 0.0))
    Qline = ports.get('Qline', meta.get('Qline0', 0.0))

    Tfltr = meta['Tfltr']
    Kp = meta['Kp']
    Ki = meta['Ki']
    Tp = meta['Tp']
    Kpg = meta['Kpg']
    Kig = meta['Kig']
    Tg = meta['Tg']
    Vfrz = meta['Vfrz']
    Ddn = meta['Ddn']
    Dup = meta['Dup']
    Qmax = meta['Qmax']
    Qmin = meta['Qmin']
    Pmax = meta['Pmax']
    Pmin = meta['Pmin']
    emax = meta['emax']
    emin = meta['emin']
    dbd1 = meta['dbd1']
    dbd2 = meta['dbd2']
    fdbd1 = meta['fdbd1']
    fdbd2 = meta['fdbd2']
    femax = meta['femax']
    femin = meta['femin']
    VCFlag = meta['VCFlag']
    RefFlag = meta['RefFlag']
    Fflag = meta['Fflag']
    PLflag = meta['PLflag']
    Vref0 = meta['Vref0']
    Qline0 = meta.get('Qline0', 0.0)
    Pline0 = meta.get('Pline0', 0.0)
    Kc = meta['Kc']

    # --- Voltage/Q measurement selection ---
    if VCFlag > 0.5:
        # Line drop compensation (simplified: use V directly)
        VCsel = V
    else:
        VCsel = Qline * Kc + V

    # s0: Voltage filter
    ds0 = (VCsel - s0_y) / Tfltr

    # s1: Qline filter
    ds1 = (Qline - s1_y) / Tfltr

    # --- Reference selection ---
    Vref = Vref0
    Qlinef = Qline0

    if RefFlag > 0.5:
        # Voltage control
        Refsel = Vref - s0_y
    else:
        # Q control
        Refsel = Qlinef - s1_y

    # Deadband
    if Refsel < dbd1:
        dbd_y = Refsel - dbd1
    elif Refsel > dbd2:
        dbd_y = Refsel - dbd2
    else:
        dbd_y = 0.0

    # Hard limit on deadband output
    enf = np.clip(dbd_y, emin, emax)

    # Freeze if V < Vfrz
    if V < Vfrz:
        enf = 0.0

    # s2: PI for reactive power/voltage
    s2_p = Kp * enf
    s2_y = s2_p + s2_xi
    s2_y_clamped = np.clip(s2_y, Qmin, Qmax)

    # Anti-windup
    if (s2_y >= Qmax and Ki * enf > 0) or \
       (s2_y <= Qmin and Ki * enf < 0):
        ds2 = 0.0
    else:
        ds2 = Ki * enf

    # --- Active power path ---
    # s4: Pline filter
    ds4 = (Pline - s4_y) / Tp

    # Frequency error
    ferr = 1.0 - f  # Frequency reference = 1.0 pu

    # Frequency deadband
    if ferr < fdbd1:
        fdbd_y = ferr - fdbd1
    elif ferr > fdbd2:
        fdbd_y = ferr - fdbd2
    else:
        fdbd_y = 0.0

    # Frequency droop
    if fdbd_y < 0:
        fdroop = fdbd_y * Ddn
    else:
        fdroop = fdbd_y * Dup

    # Pline error
    Plant_pref = Pline0
    Plerr = -s4_y + Plant_pref

    # Combined power error
    if PLflag > 0.5:
        Perr = fdroop + Plerr
    else:
        Perr = fdroop

    # Limit frequency error contribution
    Perr_limited = np.clip(Perr, femin, femax)

    # s5: PI for active power
    s5_p = Kpg * Perr_limited
    s5_y = s5_p + s5_xi
    s5_y_clamped = np.clip(s5_y, Pmin, Pmax)

    # Anti-windup
    if (s5_y >= Pmax and Kig * Perr_limited > 0) or \
       (s5_y <= Pmin and Kig * Perr_limited < 0):
        ds5 = 0.0
    else:
        ds5 = Kig * Perr_limited

    x_dot = np.zeros(5)
    x_dot[0] = ds0
    x_dot[1] = ds1
    x_dot[2] = ds2
    x_dot[3] = ds4
    x_dot[4] = ds5

    return x_dot


def repca1_output(x, ports, meta):
    """
    Compute plant controller outputs: Pext, Qext.

    Args:
        x: numpy array [s0_y, s1_y, s2_xi, s4_y, s5_xi]
        ports: dict with 'V', 'f', 'Pline', 'Qline'
        meta: dict of parameters

    Returns:
        dict with 'Pext', 'Qext'
    """
    s0_y, s1_y, s2_xi, s4_y, s5_xi = x

    Kp = meta['Kp']
    Ki = meta['Ki']
    Kpg = meta['Kpg']
    Qmax = meta['Qmax']
    Qmin = meta['Qmin']
    Pmax = meta['Pmax']
    Pmin = meta['Pmin']
    emax = meta['emax']
    emin = meta['emin']
    dbd1 = meta['dbd1']
    dbd2 = meta['dbd2']
    RefFlag = meta['RefFlag']
    Fflag = meta['Fflag']
    Vref0 = meta['Vref0']
    Qline0 = meta.get('Qline0', 0.0)

    V = ports.get('V', 1.0)
    f = ports.get('f', 1.0)

    # Qext from reactive PI (simplified: s3 lead-lag ≈ pass-through)
    if RefFlag > 0.5:
        Refsel = Vref0 - s0_y
    else:
        Refsel = Qline0 - s1_y

    if Refsel < dbd1:
        dbd_y = Refsel - dbd1
    elif Refsel > dbd2:
        dbd_y = Refsel - dbd2
    else:
        dbd_y = 0.0

    enf = np.clip(dbd_y, emin, emax)
    s2_y = np.clip(Kp * enf + s2_xi, Qmin, Qmax)
    Qext = s2_y

    # Pext from active power PI (only if Fflag=1)
    if Fflag > 0.5:
        Pext = np.clip(s5_xi, Pmin, Pmax)
    else:
        Pext = 0.0

    return {'Pext': Pext, 'Qext': Qext}


def build_repca1_core(rep_data, S_system=100.0):
    """Build REPCA1 plant controller as DynamicsCore.

    Args:
        rep_data: dict with plant controller parameters from JSON
        S_system: system base power (MVA)

    Returns:
        core: DynamicsCore object
        metadata: dict with additional info
    """
    core = DynamicsCore(label=f'REPCA1_{rep_data["idx"]}', dynamics_fn=repca1_dynamics)

    metadata = {
        'idx': rep_data['idx'],
        'ree': rep_data['ree'],
        'line': rep_data.get('line', None),
        'busf': rep_data.get('busf', None),
        'VCFlag': rep_data.get('VCFlag', 1),
        'RefFlag': rep_data.get('RefFlag', 1),
        'Fflag': rep_data.get('Fflag', 0),
        'PLflag': rep_data.get('PLflag', 0),
        'Tfltr': max(rep_data.get('Tfltr', 0.02), 0.001),
        'Kp': rep_data.get('Kp', 1.0),
        'Ki': rep_data.get('Ki', 0.1),
        'Tft': rep_data.get('Tft', 1.0),
        'Tfv': rep_data.get('Tfv', 1.0),
        'Vfrz': rep_data.get('Vfrz', 0.8),
        'Rc': rep_data.get('Rc', None),
        'Xc': rep_data.get('Xc', None),
        'Kc': rep_data.get('Kc', 1.0),
        'emax': rep_data.get('emax', 999.0),
        'emin': rep_data.get('emin', -999.0),
        'dbd1': rep_data.get('dbd1', -0.02),
        'dbd2': rep_data.get('dbd2', 0.02),
        'Qmax': rep_data.get('Qmax', 999.0),
        'Qmin': rep_data.get('Qmin', -999.0),
        'Kpg': rep_data.get('Kpg', 1.0),
        'Kig': rep_data.get('Kig', 0.1),
        'Tp': max(rep_data.get('Tp', 0.02), 0.001),
        'Tg': max(rep_data.get('Tg', 0.02), 0.001),
        'fdbd1': rep_data.get('fdbd1', -0.01),
        'fdbd2': rep_data.get('fdbd2', 0.01),
        'femax': rep_data.get('femax', 0.05),
        'femin': rep_data.get('femin', -0.05),
        'Pmax': rep_data.get('Pmax', 999.0),
        'Pmin': rep_data.get('Pmin', -999.0),
        'Ddn': rep_data.get('Ddn', 10.0),
        'Dup': rep_data.get('Dup', 10.0),
        'Vref0': 1.0,  # Set during initialization
        'Qline0': 0.0,
        'Pline0': 0.0,
    }

    # Symbolic PH structure
    s0, s1, s2, s4, s5 = core.symbols(['s0_vf', 's1_qf', 's2_qpi', 's4_pf', 's5_ppi'])
    Tfltr_sym, Tp_sym = core.symbols(['Tfltr', 'Tp'])

    H_plant = (Tfltr_sym / 2) * s0**2 + (Tfltr_sym / 2) * s1**2 + \
              s2**2 / 2 + (Tp_sym / 2) * s4**2 + s5**2 / 2
    core.add_storages([s0, s1, s2, s4, s5], H_plant)

    core.subs.update({Tfltr_sym: metadata['Tfltr'], Tp_sym: metadata['Tp']})

    core.set_metadata(metadata)
    core.n_states = 5
    core.output_fn = repca1_output
    core.component_type = "renewable"
    core.model_name = "REPCA1"

    def init_fn(V0=1.0, P0=0.35, Q0=0.1, Pline0=0.35, Qline0=0.1, **kwargs):
        """Initialize plant controller at equilibrium.
        
        CRITICAL: PI integrators must be pre-loaded to achieve zero error at equilibrium.
        For Pext output to match desired Pline0, set s5_xi = Pline0 (since proportional error = 0).
        Similarly for Qext, set s2_xi appropriately.
        """
        metadata['Vref0'] = V0
        metadata['Qline0'] = Qline0
        metadata['Pline0'] = Pline0
        
        # At equilibrium:
        # - All filters = measured values
        # - PI integrators pre-loaded so outputs match setpoints with zero proportional error
        
        # For active power: Pext = s5_xi when Perr=0 (Kpg*Perr=0)
        # We want Pext = Pline0, so s5_xi = Pline0
        s5_xi_eq = Pline0
        
        # For reactive power: Qext = s2_xi when enf=0 (Kp*enf=0)
        # If V=Vref0 at equilibrium, enf=0, so Qext = s2_xi
        # For voltage control to be satisfied, we want Qext = Qline0
        s2_xi_eq = Qline0
        
        return np.array([V0, Qline0, s2_xi_eq, Pline0, s5_xi_eq])

    core.init_fn = init_fn

    return core, metadata
