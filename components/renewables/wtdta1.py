"""
WTDTA1 Wind Turbine Drive-Train Model - Port-Hamiltonian Formulation

Two-mass drive train model with turbine inertia, generator inertia, and shaft spring.

PH Structure:
    States: x = [theta_tw, p_t, p_g]
        theta_tw: shaft twist angle (rad)
        p_t: turbine angular momentum (pu)
        p_g: generator angular momentum (pu)

    Hamiltonian: H = p_t^2/(2*J_t) + p_g^2/(2*J_g) + (1/2)*K_shaft*theta_tw^2

    Dissipation: D_shaft*(omega_t - omega_g), DAMP*(omega_g - omega_0)

    Ports:
        Inputs: Pm (mechanical power from aerodynamics), Pe (electrical power from converter)
        Outputs: wt (turbine speed), wg (generator speed)

Reference: WECC WTDTA1 model specification
"""
from utils.pyphs_core import DynamicsCore
import numpy as np


def wtdta1_dynamics(x, ports, meta):
    """
    Numerical dynamics for WTDTA1 drive-train.

    Args:
        x: numpy array of 3 states [theta_tw, p_t, p_g]
        ports: dict with keys {'Pm', 'Pe'}
        meta: dict of drive train parameters

    Returns:
        x_dot: numpy array of 3 state derivatives
    """
    theta_tw, p_t, p_g = x

    Pm = ports.get('Pm', meta['Pe0'])
    Pe = ports.get('Pe', meta['Pe0'])

    Jt = meta['Jt']       # Turbine inertia = 2 * Htfrac * H
    Jg = meta['Jg']       # Generator inertia = 2 * (1-Htfrac) * H
    Kshaft = meta['Kshaft']
    Dshaft = meta['Dshaft']
    DAMP = meta['DAMP']
    w0 = meta['w0']

    # Speeds from momenta
    wt = p_t / Jt
    wg = p_g / Jg

    # Prevent division by zero
    wt_safe = max(wt, 0.01)
    wg_safe = max(wg, 0.01)

    # Shaft torque (spring)
    T_shaft = Kshaft * theta_tw

    # Shaft damping torque
    pd = Dshaft * (wt - wg)

    # Mechanical torque on turbine (Pm/wt)
    Tm = Pm / wt_safe

    # Electrical torque on generator (Pe/wg)
    Te = Pe / wg_safe

    x_dot = np.zeros(3)

    # PH structure: dx/dt = (J - R) * dH/dx + B * u
    # dH/dx = [K_shaft * theta_tw, p_t/J_t, p_g/J_g] = [T_shaft, wt, wg]

    # d(theta_tw)/dt = wt - wg (twist rate)
    x_dot[0] = wt - wg

    # d(p_t)/dt = Tm - T_shaft - pd (turbine momentum)
    x_dot[1] = Tm - T_shaft - pd

    # d(p_g)/dt = T_shaft + pd - Te - DAMP*(wg - w0) (generator momentum)
    x_dot[2] = T_shaft + pd - Te - DAMP * (wg - w0)

    return x_dot


def wtdta1_output(x, ports, meta):
    """
    Compute drive train outputs: turbine and generator speeds.

    Args:
        x: numpy array [theta_tw, p_t, p_g]
        ports: dict (unused)
        meta: dict of parameters

    Returns:
        dict with 'wt' and 'wg'
    """
    theta_tw, p_t, p_g = x
    wt = p_t / meta['Jt']
    wg = p_g / meta['Jg']
    return {'wt': wt, 'wg': wg}


def build_wtdta1_core(dt_data, S_system=100.0):
    """Build WTDTA1 drive train as DynamicsCore.

    Args:
        dt_data: dict with drive train parameters from JSON
        S_system: system base power (MVA)

    Returns:
        core: DynamicsCore object
        metadata: dict with additional info
    """
    core = DynamicsCore(label=f'WTDTA1_{dt_data["idx"]}', dynamics_fn=wtdta1_dynamics)

    H = dt_data['H']
    DAMP = dt_data.get('DAMP', 0.0)
    Htfrac = dt_data.get('Htfrac', 0.5)
    Freq1 = dt_data.get('Freq1', 1.0)
    Dshaft = dt_data.get('Dshaft', 1.0)
    w0 = dt_data.get('w0', 1.0)

    # Inertia constants (on system base, 2H format)
    Jt = 2.0 * Htfrac * H
    Jg = 2.0 * (1.0 - Htfrac) * H

    # Shaft stiffness: Kshaft = Jt * Jg * Freq1^2 / (2*H)
    # (Freq1 in p.u. Hz, so (2*pi*Freq1)^2 reduces to Freq1^2 in p.u.)
    Kshaft = Jt * Jg * 0.5 * Freq1 * Freq1 / H

    # Symbolic PH structure
    theta_tw, p_t, p_g = core.symbols(['theta_tw', 'p_t', 'p_g'])
    Jt_sym, Jg_sym, K_sym = core.symbols(['Jt', 'Jg', 'Kshaft'])

    H_mech = p_t**2 / (2 * Jt_sym) + p_g**2 / (2 * Jg_sym)
    H_spring = K_sym * theta_tw**2 / 2
    H_total = H_mech + H_spring

    core.add_storages([theta_tw, p_t, p_g], H_total)

    # Dissipations
    w_shaft = core.symbols('w_shaft')
    D_shaft_sym = core.symbols('Dshaft')
    z_shaft = D_shaft_sym * w_shaft

    w_damp = core.symbols('w_damp')
    DAMP_sym = core.symbols('DAMP')
    z_damp = DAMP_sym * w_damp

    core.add_dissipations([w_shaft, w_damp], [z_shaft, z_damp])

    # Ports
    Pm_in, Pe_in = core.symbols(['Pm_in', 'Pe_in'])
    wt_out, wg_out = core.symbols(['wt_out', 'wg_out'])
    core.add_ports([Pm_in, Pe_in], [wt_out, wg_out])

    core.subs.update({
        Jt_sym: Jt, Jg_sym: Jg, K_sym: Kshaft,
        D_shaft_sym: Dshaft, DAMP_sym: DAMP,
    })

    metadata = {
        'idx': dt_data['idx'],
        'ree': dt_data['ree'],
        'H': H, 'Jt': Jt, 'Jg': Jg,
        'Kshaft': Kshaft, 'Dshaft': Dshaft,
        'DAMP': DAMP, 'Htfrac': Htfrac,
        'Freq1': Freq1, 'w0': w0,
        'Pe0': 0.0,  # Set during initialization
    }

    core.set_metadata(metadata)
    core.n_states = 3
    core.output_fn = wtdta1_output
    core.component_type = "renewable"
    core.model_name = "WTDTA1"

    def init_fn(Pe0=0.35, w0_init=1.0, **kwargs):
        """Initialize drive train at equilibrium."""
        wt0 = w0_init
        wg0 = w0_init
        p_t0 = Jt * wt0
        p_g0 = Jg * wg0
        theta_tw0 = Pe0 / (wg0 * Kshaft) if Kshaft > 0 else 0.0
        return np.array([theta_tw0, p_t0, p_g0])

    core.init_fn = init_fn

    return core, metadata
