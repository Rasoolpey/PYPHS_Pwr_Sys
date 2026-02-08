"""EXDC2 Exciter Model - Full IEEE Standard Implementation"""
import sys
sys.path.insert(0, '/home/claude')
from utils.pyphs_core import DynamicsCore
import numpy as np


def _compute_saturation(Efd, meta):
    """
    Compute saturation factor SE(Efd) from IEEE saturation data.

    Uses quadratic saturation model: SE(E) = B * (E - A)^2 / E
    where A, B are derived from (E1, SE1) and (E2, SE2).

    When E1=0 or E2=0, saturation is disabled (returns 0).

    Args:
        Efd: Field voltage magnitude
        meta: dict with E1, SE1, E2, SE2

    Returns:
        SE: Saturation factor at Efd
    """
    E1 = meta.get('E1', 0.0)
    SE1 = meta.get('SE1', 0.0)
    E2 = meta.get('E2', 1.0)
    SE2 = meta.get('SE2', 1.0)

    # Disable saturation when E1=0 (common convention)
    if E1 == 0 or E2 == 0 or abs(E2 - E1) < 1e-10:
        return 0.0

    Efd_abs = abs(Efd)
    if Efd_abs < 0.75 * E1:
        return 0.0

    # Quadratic saturation model: SE(E) = B * (E - A)^2 / E
    # Solve for A, B from two data points
    sqrt_SE1E1 = np.sqrt(SE1 * E1) if SE1 * E1 > 0 else 0.0
    sqrt_SE2E2 = np.sqrt(SE2 * E2) if SE2 * E2 > 0 else 0.0

    if abs(sqrt_SE2E2 - sqrt_SE1E1) < 1e-10:
        return 0.0

    A = (E1 * sqrt_SE2E2 - E2 * sqrt_SE1E1) / (sqrt_SE2E2 - sqrt_SE1E1)
    B_numer = sqrt_SE1E1
    B_denom = E1 - A

    if abs(B_denom) < 1e-10:
        return 0.0

    B = (B_numer / B_denom) ** 2

    if Efd_abs <= A:
        return 0.0

    return B * (Efd_abs - A) ** 2 / Efd_abs


def exdc2_dynamics(x, ports, meta):
    """
    Full IEEE EXDC2 exciter dynamics.

    States: [vm, vr, efd, xf]
        vm:  Voltage measurement (filtered terminal voltage)
        vr:  Amplifier/regulator output (with VRMAX/VRMIN limits)
        efd: Exciter output (field voltage)
        xf:  Rate feedback filter state

    The signal path is:
        Vt -> [TR filter] -> vm
        Verr = Vref - vm - Vf
        Verr -> [KA/(1+sTA)] -> vr (with VRMAX/VRMIN limits)
        vr -> [1/(KE+SE+sTE)] -> efd
        efd -> [sKF1/(1+sTF1)] -> Vf (rate feedback)

    Args:
        x: numpy array of 4 states [vm, vr, efd, xf]
        ports: dict with key 'Vt' (terminal voltage)
        meta: dict of exciter parameters

    Returns:
        x_dot: numpy array of 4 state derivatives
    """
    # Extract states
    vm, vr, efd, xf = x

    # Extract ports
    Vt = ports.get('Vt', 1.0)

    # Extract parameters
    TR = meta['TR']
    TA = meta['TA']
    KA = meta['KA']
    KE = meta['KE']
    TE = meta['TE']
    KF = meta['KF']
    TF1 = meta['TF1']
    Vref = meta['Vref']
    VRMAX = meta['VRMAX']
    VRMIN = meta['VRMIN']

    # State derivatives
    x_dot = np.zeros(4)

    # 1. Voltage measurement filter
    x_dot[0] = (Vt - vm) / TR

    # 2. Rate feedback signal: Vf = KF/TF1 * (efd - xf)
    Vf = KF * (efd - xf) / TF1

    # 3. Voltage error (includes rate feedback)
    Verr = Vref - vm - Vf

    # 4. Amplifier/regulator with anti-windup limits
    vr_unlimited = (KA * Verr - vr) / TA
    if vr >= VRMAX and vr_unlimited > 0:
        x_dot[1] = 0.0
    elif vr <= VRMIN and vr_unlimited < 0:
        x_dot[1] = 0.0
    else:
        x_dot[1] = vr_unlimited

    # 5. Exciter with saturation
    SE_efd = _compute_saturation(efd, meta)
    x_dot[2] = (vr - (KE + SE_efd) * efd) / TE

    # 6. Rate feedback filter state
    x_dot[3] = (efd - xf) / TF1

    return x_dot


def exdc2_output(x, ports, meta):
    """
    Compute exciter output Efd.

    Args:
        x: numpy array of states [vm, vr, efd, xf]
        ports: dict (not used for output)
        meta: dict of parameters

    Returns:
        Efd: field voltage (efd state, index 2)
    """
    vm, vr, efd, xf = x
    return efd


def build_exdc2_core(exc_data):
    """Build EXDC2 exciter as DynamicsCore

    Args:
        exc_data: dict with exciter parameters

    Returns:
        core: DynamicsCore object with dynamics method
        metadata: dict with additional info
    """
    core = DynamicsCore(label=f'EXDC2_{exc_data["idx"]}', dynamics_fn=exdc2_dynamics)

    # Extract parameters
    TR = max(exc_data['TR'], 0.001)
    KA = min(exc_data['KA'], 200.0)
    TA = max(exc_data['TA'], 0.001)
    KE = exc_data['KE']
    TE = max(exc_data['TE'], 0.001)
    KF = exc_data.get('KF1', 0.0754)
    TF1 = max(exc_data['TF1'], 0.001)
    TC = exc_data['TC']
    TB = exc_data['TB']

    Vref = 1.0

    # States: [vm, vr, efd, xf]
    vm, vr, efd_sym, xf = core.symbols(['vm', 'vr', 'efd', 'xf'])

    # Parameters
    TR_sym, KA_sym, TA_sym = core.symbols(['TR_exc', 'KA_exc', 'TA_exc'])
    KE_sym, TE_sym = core.symbols(['KE_exc', 'TE_exc'])
    KF_sym, TF1_sym = core.symbols(['KF_exc', 'TF1_exc'])
    TC_sym, TB_sym = core.symbols(['TC_exc', 'TB_exc'])
    Vref_sym = core.symbols('Vref_exc')

    # Hamiltonian
    H_exc = (TR_sym/2)*vm**2 + (TA_sym/2)*vr**2 + (TE_sym/2)*efd_sym**2 + (TF1_sym/2)*xf**2

    core.add_storages([vm, vr, efd_sym, xf], H_exc)

    # Dissipations
    w_vm = core.symbols('w_vm')
    z_vm = w_vm / TR_sym

    w_vr = core.symbols('w_vr')
    z_vr = w_vr / TA_sym

    w_efd = core.symbols('w_efd')
    z_efd = w_efd / TE_sym

    w_xf = core.symbols('w_xf')
    z_xf = w_xf / TF1_sym

    core.add_dissipations([w_vm, w_vr, w_efd, w_xf],
                         [z_vm, z_vr, z_efd, z_xf])

    # Ports: Input [Vt_in], Output [Efd_out]
    Vt_in = core.symbols('Vt_in')
    Efd_out = core.symbols('Efd_out')

    core.add_ports([Vt_in], [Efd_out])

    # Parameter substitutions
    core.subs.update({
        TR_sym: TR,
        KA_sym: KA,
        TA_sym: TA,
        KE_sym: KE,
        TE_sym: TE,
        KF_sym: KF,
        TF1_sym: TF1,
        TC_sym: TC,
        TB_sym: TB,
        Vref_sym: Vref
    })

    # Metadata
    metadata = {
        'idx': exc_data['idx'],
        'syn': exc_data['syn'],
        'VRMAX': exc_data['VRMAX'],
        'VRMIN': exc_data['VRMIN'],
        'E1': exc_data['E1'],
        'SE1': exc_data['SE1'],
        'E2': exc_data['E2'],
        'SE2': exc_data['SE2'],
        'TR': TR,
        'TA': TA,
        'TE': TE,
        'TF1': TF1,
        'KA': KA,
        'KE': KE,
        'KF': KF,
        'TC': TC,
        'TB': TB,
        'Vref': Vref
    }

    # Set metadata on core for dynamics computation
    core.set_metadata(metadata)

    # Set component interface attributes
    core.n_states = 4
    core.output_fn = exdc2_output
    core.init_fn = lambda Efd_eq, V_mag, **kwargs: np.array([V_mag, Efd_eq, Efd_eq, Efd_eq])
    core.component_type = "exciter"
    core.model_name = "EXDC2"

    return core, metadata
