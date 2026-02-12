"""
WTTQA1 Wind Turbine Torque/Power Reference Control - Port-Hamiltonian Formulation

Generates power reference (Pref) for the electrical control model (REECA1)
based on optimal speed tracking using a piecewise Pe-to-speed lookup table.

PH Structure:
    States: x = [s1_y, s2_y, pi_xi]
        s1_y: filtered active power (Pe)
        s2_y: filtered speed reference
        pi_xi: PI controller integrator state

    Hamiltonian: H = (Tp/2)*s1^2 + (Twref/2)*s2^2 + (1/2)*pi^2

    Ports:
        Inputs: Pe (from converter), wg (from drive train)
        Outputs: Pref (to REECA1)

Reference: WECC WTTQA1 model specification
"""
from utils.pyphs_core import DynamicsCore
import numpy as np


def wttqa1_piecewise_speed(Pe_filt, meta):
    """
    Piecewise linear Pe-to-optimal-speed lookup.

    Args:
        Pe_filt: filtered active power
        meta: dict with p1,sp1,p2,sp2,p3,sp3,p4,sp4

    Returns:
        optimal speed reference
    """
    p1, sp1 = meta['p1'], meta['sp1']
    p2, sp2 = meta['p2'], meta['sp2']
    p3, sp3 = meta['p3'], meta['sp3']
    p4, sp4 = meta['p4'], meta['sp4']

    if Pe_filt <= p1:
        return sp1
    elif Pe_filt <= p2:
        return sp1 + (Pe_filt - p1) * (sp2 - sp1) / (p2 - p1)
    elif Pe_filt <= p3:
        return sp2 + (Pe_filt - p2) * (sp3 - sp2) / (p3 - p2)
    elif Pe_filt <= p4:
        return sp3 + (Pe_filt - p3) * (sp4 - sp3) / (p4 - p3)
    else:
        return sp4


def wttqa1_dynamics(x, ports, meta):
    """
    Numerical dynamics for WTTQA1 torque controller.

    Args:
        x: numpy array of 3 states [s1_y, s2_y, pi_xi]
        ports: dict with keys {'Pe', 'wg'}
        meta: dict of controller parameters

    Returns:
        x_dot: numpy array of 3 state derivatives
    """
    s1_y, s2_y, pi_xi = x

    Pe = ports.get('Pe', meta.get('Pref0', 0.0))
    wg = ports.get('wg', 1.0)

    Tp = meta['Tp']
    Twref = meta['Twref']
    Kip = meta['Kip']
    Kpp = meta['Kpp']
    Temax = meta['Temax']
    Temin = meta['Temin']
    Tflag = meta['Tflag']
    Pref0 = meta.get('Pref0', 0.0)

    # s1: Pe filter
    ds1 = (Pe - s1_y) / Tp

    # Piecewise optimal speed from filtered Pe
    fPe_y = wttqa1_piecewise_speed(s1_y, meta)

    # s2: speed reference filter
    ds2 = (fPe_y - s2_y) / Twref

    # PI controller input selection
    if Tflag > 0.5:
        # Power error mode
        wg_safe = max(wg, 0.01)
        Tsel = (Pe - Pref0) / wg_safe
    else:
        # Speed error mode
        Tsel = s2_y - wg

    # PI controller
    pi_p = Kpp * Tsel
    pi_y = pi_p + pi_xi

    # Clamp PI output
    pi_y_clamped = np.clip(pi_y, Temin, Temax)

    # Anti-windup
    if (pi_y >= Temax and Kip * Tsel > 0) or \
       (pi_y <= Temin and Kip * Tsel < 0):
        dpi = 0.0
    else:
        dpi = Kip * Tsel

    x_dot = np.zeros(3)
    x_dot[0] = ds1
    x_dot[1] = ds2
    x_dot[2] = dpi

    return x_dot


def wttqa1_output(x, ports, meta):
    """
    Compute torque controller output: Pref.

    Args:
        x: numpy array [s1_y, s2_y, pi_xi]
        ports: dict with 'wg'
        meta: dict of parameters

    Returns:
        dict with 'Pref', 'wref' (optimal speed reference)
    """
    s1_y, s2_y, pi_xi = x
    wg = ports.get('wg', 1.0)

    Kpp = meta['Kpp']
    Temax = meta['Temax']
    Temin = meta['Temin']
    Tflag = meta['Tflag']
    Pref0 = meta.get('Pref0', 0.0)

    # PI output
    if Tflag > 0.5:
        wg_safe = max(wg, 0.01)
        Tsel = (ports.get('Pe', Pref0) - Pref0) / wg_safe
    else:
        Tsel = s2_y - wg

    pi_y = np.clip(Kpp * Tsel + pi_xi, Temin, Temax)

    # Pref = PI_y * wg (torque * speed = power)
    Pref = pi_y * wg

    # Optimal speed from piecewise lookup
    fPe_y = wttqa1_piecewise_speed(s1_y, meta)

    return {'Pref': Pref, 'wref': fPe_y}


def build_wttqa1_core(tq_data, S_system=100.0):
    """Build WTTQA1 torque controller as DynamicsCore.

    Args:
        tq_data: dict with torque controller parameters from JSON
        S_system: system base power (MVA)

    Returns:
        core: DynamicsCore object
        metadata: dict with additional info
    """
    core = DynamicsCore(label=f'WTTQA1_{tq_data["idx"]}', dynamics_fn=wttqa1_dynamics)

    metadata = {
        'idx': tq_data['idx'],
        'rep': tq_data['rep'],
        'Kip': tq_data.get('Kip', 0.1),
        'Kpp': tq_data.get('Kpp', 0.0),
        'Tp': max(tq_data.get('Tp', 0.05), 0.001),
        'Twref': max(tq_data.get('Twref', 30.0), 0.001),
        'Temax': tq_data.get('Temax', 1.2),
        'Temin': tq_data.get('Temin', 0.0),
        'Tflag': tq_data.get('Tflag', 0),
        'p1': tq_data.get('p1', 0.2),
        'sp1': tq_data.get('sp1', 0.58),
        'p2': tq_data.get('p2', 0.4),
        'sp2': tq_data.get('sp2', 0.72),
        'p3': tq_data.get('p3', 0.6),
        'sp3': tq_data.get('sp3', 0.86),
        'p4': tq_data.get('p4', 0.8),
        'sp4': tq_data.get('sp4', 1.0),
        'Pref0': 0.0,  # Set during initialization
    }

    # Symbolic PH structure
    s1, s2, pi_xi = core.symbols(['s1_pe', 's2_wref', 'pi_tq'])
    Tp_sym, Twref_sym = core.symbols(['Tp', 'Twref'])

    H_tq = (Tp_sym / 2) * s1**2 + (Twref_sym / 2) * s2**2 + pi_xi**2 / 2
    core.add_storages([s1, s2, pi_xi], H_tq)

    core.subs.update({Tp_sym: metadata['Tp'], Twref_sym: metadata['Twref']})

    core.set_metadata(metadata)
    core.n_states = 3
    core.output_fn = wttqa1_output
    core.component_type = "renewable"
    core.model_name = "WTTQA1"

    def init_fn(Pe0=0.35, wg0=1.0, **kwargs):
        """Initialize torque controller at equilibrium."""
        metadata['Pref0'] = Pe0
        # Calculate optimal speed reference from piecewise lookup
        fPe_y = wttqa1_piecewise_speed(Pe0, metadata)
        
        # Initialization strategy depends on control mode
        Tflag = metadata.get('Tflag', 0)
        if Tflag > 0.5:
            # POWER MODE: Tsel = (Pe - Pref0) / wg
            # At equilibrium: Pe = Pref0, so Tsel = 0 regardless of s2_y
            # Set s2_y = fPe_y to eliminate filter transient (ds2 = 0)
            # Actual turbine speed wg can differ - doesn't affect equilibrium
            s2_y0 = fPe_y
        else:
            # SPEED MODE: Tsel = s2_y - wg
            # At equilibrium need Tsel = 0, so s2_y = wg0
            # Accept filter transient: ds2 = (fPe_y - wg0) / Twref
            s2_y0 = wg0
        
        pi_xi0 = Pe0 / max(wg0, 0.01)  # torque to produce Pe0
        pi_xi0 = np.clip(pi_xi0, metadata['Temin'], metadata['Temax'])
        return np.array([Pe0, s2_y0, pi_xi0])

    core.init_fn = init_fn

    return core, metadata
