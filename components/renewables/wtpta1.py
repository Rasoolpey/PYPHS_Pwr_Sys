"""
WTPTA1 Wind Turbine Pitch Control Model - Port-Hamiltonian Formulation

Pitch controller that adjusts blade pitch angle to regulate turbine speed
and limit power output. Uses two PI controllers: one for speed deviation,
one for power compensation.

PH Structure:
    States: x = [piw_xi, pic_xi, lg_y]
        piw_xi: speed PI controller integrator
        pic_xi: power compensation PI controller integrator
        lg_y: lag output (pitch angle theta, in radians)

    Hamiltonian: H = (1/2)*piw^2 + (1/2)*pic^2 + (Tp/2)*lg^2

    Ports:
        Inputs: wt (turbine speed), Pord (power order), Pref (power reference)
        Outputs: theta (pitch angle in radians)

Reference: WECC WTPTA1 model specification
"""
from utils.pyphs_core import DynamicsCore
import numpy as np


def wtpta1_dynamics(x, ports, meta):
    """
    Numerical dynamics for WTPTA1 pitch controller.

    Args:
        x: numpy array of 3 states [piw_xi, pic_xi, lg_y]
        ports: dict with keys {'wt', 'Pord', 'Pref'}
        meta: dict of pitch control parameters

    Returns:
        x_dot: numpy array of 3 state derivatives
    """
    piw_xi, pic_xi, lg_y = x

    wt = ports.get('wt', 1.0)
    Pord = ports.get('Pord', meta.get('Pref0', 0.0))
    Pref = ports.get('Pref', meta.get('Pref0', 0.0))

    Kiw = meta['Kiw']
    Kpw = meta['Kpw']
    Kic = meta['Kic']
    Kpc = meta['Kpc']
    Kcc = meta['Kcc']
    Tp = meta['Tp']
    thmax = meta['thmax_rad']
    thmin = meta['thmin_rad']
    dthmax = meta['dthmax_rad']
    dthmin = meta['dthmin_rad']
    wref0 = meta.get('wref0', 1.0)

    # --- PIc: Compensation PI for active power difference ---
    pic_input = Pord - Pref
    pic_p = Kpc * pic_input
    pic_y = pic_p + pic_xi
    pic_y_clamped = np.clip(pic_y, thmin, thmax)

    # Anti-windup for PIc
    if (pic_y >= thmax and Kic * pic_input > 0) or \
       (pic_y <= thmin and Kic * pic_input < 0):
        dpic = 0.0
    else:
        dpic = Kic * pic_input

    # --- PIw: Speed PI for speed + power deviation ---
    piw_input = Kcc * (Pord - Pref) + wt - wref0
    piw_p = Kpw * piw_input
    piw_y = piw_p + piw_xi
    piw_y_clamped = np.clip(piw_y, thmin, thmax)

    # Anti-windup for PIw
    if (piw_y >= thmax and Kiw * piw_input > 0) or \
       (piw_y <= thmin and Kiw * piw_input < 0):
        dpiw = 0.0
    else:
        dpiw = Kiw * piw_input

    # --- LG: Lag with anti-windup and rate limits ---
    lg_input = piw_y_clamped + pic_y_clamped
    lg_error = lg_input - lg_y
    lg_rate = lg_error / Tp

    # Apply rate limits
    lg_rate = np.clip(lg_rate, dthmin, dthmax)

    # Apply position limits (anti-windup)
    if lg_y >= thmax and lg_rate > 0:
        lg_rate = 0.0
    elif lg_y <= thmin and lg_rate < 0:
        lg_rate = 0.0

    x_dot = np.zeros(3)
    x_dot[0] = dpiw
    x_dot[1] = dpic
    x_dot[2] = lg_rate

    return x_dot


def wtpta1_output(x, ports, meta):
    """
    Compute pitch controller output: theta.

    Args:
        x: numpy array [piw_xi, pic_xi, lg_y]
        ports: dict (unused)
        meta: dict of parameters

    Returns:
        dict with 'theta' (pitch angle in radians)
    """
    return {'theta': x[2]}


def build_wtpta1_core(pt_data, S_system=100.0):
    """Build WTPTA1 pitch controller as DynamicsCore.

    Args:
        pt_data: dict with pitch control parameters from JSON
        S_system: system base power (MVA)

    Returns:
        core: DynamicsCore object
        metadata: dict with additional info
    """
    core = DynamicsCore(label=f'WTPTA1_{pt_data["idx"]}', dynamics_fn=wtpta1_dynamics)

    # Convert limits from degrees to radians
    thmax_rad = np.radians(pt_data.get('thmax', 30.0))
    thmin_rad = np.radians(pt_data.get('thmin', 0.0))
    dthmax_rad = np.radians(pt_data.get('dthmax', 5.0))
    dthmin_rad = np.radians(pt_data.get('dthmin', -5.0))

    metadata = {
        'idx': pt_data['idx'],
        'rea': pt_data['rea'],
        'Kiw': pt_data.get('Kiw', 0.1),
        'Kpw': pt_data.get('Kpw', 0.0),
        'Kic': pt_data.get('Kic', 0.1),
        'Kpc': pt_data.get('Kpc', 0.0),
        'Kcc': pt_data.get('Kcc', 0.0),
        'Tp': max(pt_data.get('Tp', 0.3), 0.001),
        'thmax_rad': thmax_rad,
        'thmin_rad': thmin_rad,
        'dthmax_rad': dthmax_rad,
        'dthmin_rad': dthmin_rad,
        'wref0': 1.0,  # Set during initialization from WTTQA1
        'Pref0': 0.0,
    }

    # Symbolic PH structure
    piw, pic, lg = core.symbols(['piw_xi', 'pic_xi', 'lg_theta'])
    Tp_sym = core.symbols('Tp')

    H_pitch = piw**2 / 2 + pic**2 / 2 + (Tp_sym / 2) * lg**2
    core.add_storages([piw, pic, lg], H_pitch)

    core.subs.update({Tp_sym: metadata['Tp']})

    core.set_metadata(metadata)
    core.n_states = 3
    core.output_fn = wtpta1_output
    core.component_type = "renewable"
    core.model_name = "WTPTA1"

    def init_fn(wt0=1.0, Pref0=0.35, **kwargs):
        """Initialize pitch controller at equilibrium."""
        metadata['wref0'] = wt0
        metadata['Pref0'] = Pref0
        # At equilibrium: pitch angle = theta0 (usually 0), PI integrators = 0
        theta0_rad = kwargs.get('theta0_rad', 0.0)
        return np.array([0.0, 0.0, theta0_rad])

    core.init_fn = init_fn

    return core, metadata
