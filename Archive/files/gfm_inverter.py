"""
GFM_INVERTER - Passive Grid-Forming Inverter Controller
Port-Hamiltonian Formulation

Based on:
  Kong, L., Xue, Y., Qiao, L., & Wang, F. (2024).
  "Control Design of Passive Grid-Forming Inverters in Port-Hamiltonian Framework"
  IEEE Transactions on Power Electronics, 39(1), 332-345.

State representation (polar form for correct equilibrium):
  x = [u_mag, theta, Pf, Qf]
    u_mag  : terminal voltage magnitude |u| (pu)
    theta  : virtual oscillator phase angle (rad)
    Pf     : LPF-filtered active power (pu)
    Qf     : LPF-filtered reactive power (pu)

PH Structure (closed-loop, eq.14 in paper):
    H = H_P + H_C
    H_P = (1/2)*Cf*u^2  (capacitor energy)
    H_C = (1/2)*Phi^2/eps^2  where Phi = (1/2)*Cf*(u^2 - u_ref^2)

    Passivity: dH/dt <= ig^T * u  (power absorbed from grid)
    The "pumping-or-damping" block in the reactive power loop ensures R >= 0

Control loops (Section II-B):
  Outer Power Loop:
    dtheta/dt = omega0 + mp*(Pref - Pf)          (P-f droop)
    du/dt = xi1*(u_ref^2 - u^2)*u + xi2_signed*(Qref/u_ref^2 - Qf/u^2)*u
            (Q-u oscillator amplitude with pumping-or-damping)

Parameters (JSON):
  Sn, bus        : rating and bus number
  Lf, Cf, Rf    : LC filter (pu)
  u_ref          : voltage reference magnitude (pu)
  omega0         : synchronous frequency reference (pu = 1.0 for 60 Hz)
  xi1            : voltage convergence gain (amplitude regulation)
  xi2            : reactive power pump/damp gain (sign-indefinite)
  mp             : active power - frequency droop gain (pu/pu)
  omega_lpf      : LPF cutoff for P/Q measurement (rad/s in pu time)
  Pref, Qref     : power references (pu)
  hysteresis     : hysteresis band for pump/damp switching
  Imax           : current limit (pu)
"""

from utils.pyphs_core import DynamicsCore
import numpy as np


def gfm_inverter_dynamics(x, ports, meta):
    """
    GFM inverter dynamics - polar voltage representation.

    States:
      x[0] = u_mag  : |u(t)| voltage magnitude (pu)
      x[1] = theta  : virtual oscillator angle (rad)
      x[2] = Pf     : filtered active power (pu)
      x[3] = Qf     : filtered reactive power (pu)

    Ports:
      'ig_a', 'ig_b' : grid current components (pu, αβ frame)
                       Alternatively 'P_grid', 'Q_grid' if already computed

    Returns:
      numpy array shape (4,)
    """
    u_mag, theta, Pf, Qf = x

    # Current injected into the bus from external network
    ig_a = ports.get('ig_a', 0.0)
    ig_b = ports.get('ig_b', 0.0)

    # ---------- Parameters ----------
    Cf        = meta['Cf']
    u_ref     = meta['u_ref']
    omega0    = meta['omega0']
    xi1       = meta['xi1']
    xi2_abs   = abs(meta['xi2'])
    mp        = meta['mp']
    omega_lpf = meta['omega_lpf']
    Pref      = meta['Pref']
    Qref      = meta['Qref']
    hysteresis = meta.get('hysteresis', 0.01)
    eps       = meta['eps']

    # ---------- Instantaneous power from αβ currents ----------
    # GFM outputs: u_a = u_mag*cos(theta), u_b = u_mag*sin(theta)
    u_a = u_mag * np.cos(theta)
    u_b = u_mag * np.sin(theta)
    P_inst = u_a * ig_a + u_b * ig_b
    Q_inst = u_b * ig_a - u_a * ig_b

    # ---------- Power LPF ----------
    dPf = omega_lpf * (P_inst - Pf)
    dQf = omega_lpf * (Q_inst - Qf)

    # ---------- Pumping-or-damping: sign selection for xi2 ----------
    u2     = u_mag**2
    u2_ref = u_ref**2
    Phi    = 0.5 * Cf * (u2 - u2_ref)       # storage variable, zero at u=u_ref

    if u2 > 1e-6:
        Q_ratio  = Qref / u2_ref - Qf / u2
        # Passivity condition (eq.18): sgn(xi2) must oppose sgn(Q_ratio * Phi)
        signal   = Q_ratio * Phi
        if signal > hysteresis:
            sgn_xi2 = -1.0
        elif signal < -hysteresis:
            sgn_xi2 = +1.0
        else:
            sgn_xi2 = meta.get('_sgn_xi2', 1.0)   # hold last value
        meta['_sgn_xi2'] = sgn_xi2
        xi2 = sgn_xi2 * xi2_abs
    else:
        xi2 = xi2_abs
        Q_ratio = 0.0

    # ---------- Voltage amplitude dynamics (eq. 10 and the u-dot equation) ----------
    # From eq.(19) with the outer loop only (neglecting CbI eps term at slow time scale):
    #   du/dt = A/Cf * u   (in magnitude, since rotation is handled by theta)
    # where A = Cf*xi1*(u_ref^2 - u^2) + Cf*xi2*(Qref/u_ref^2 - Qf/u^2)
    A  = Cf * xi1 * (u2_ref - u2) + Cf * xi2 * Q_ratio
    du_mag = (A / Cf) * u_mag    # magnitude: du/dt = A/Cf * |u|

    # ---------- Phase angle / frequency (eq. 11) ----------
    # Omega = omega0*Cf + xi3*Cf*(Pref/u_ref^2 - Pf/u^2)
    # In the paper xi3 = ξ3 and the frequency deviation is Omega/Cf - omega0
    # Here we use droop form: dθ/dt = omega0 + mp*(Pref - Pf)
    # Note: at equilibrium Pf=Pref so dθ/dt = omega0 (steady rotation)
    if u2 > 1e-6:
        omega = omega0 + mp * (Pref / u2_ref - Pf / u2) * u2_ref
    else:
        omega = omega0
    dtheta = omega

    x_dot = np.array([du_mag, dtheta, dPf, dQf])
    return x_dot


def gfm_inverter_output(x, ports, meta):
    """
    Compute GFM inverter outputs for network interface.

    Returns:
      dict: Pe, Qe, u_a, u_b (αβ terminal voltage), V, theta, omega
    """
    u_mag, theta, Pf, Qf = x
    ig_a = ports.get('ig_a', 0.0)
    ig_b = ports.get('ig_b', 0.0)

    u_a = u_mag * np.cos(theta)
    u_b = u_mag * np.sin(theta)
    Pe  = u_a * ig_a + u_b * ig_b
    Qe  = u_b * ig_a - u_a * ig_b

    u2 = u_mag**2
    u2_ref = meta['u_ref']**2
    omega = (meta['omega0'] +
             meta['mp'] * (meta['Pref'] / u2_ref - Pf / u2) * u2_ref
             if u2 > 1e-6 else meta['omega0'])

    return {
        'Pe'   : Pe,
        'Qe'   : Qe,
        'u_a'  : u_a,
        'u_b'  : u_b,
        'V'    : u_mag,
        'theta': theta,
        'omega': omega,
        # For network interface compatibility with REGCA1:
        'Ipout': Pe / max(u_mag, 0.01),
        'Iqout': Qe / max(u_mag, 0.01),
    }


def build_gfm_inverter_core(gfm_data, S_system=100.0):
    """
    Build the GFM inverter as a DynamicsCore object.

    Parameters (all in per-unit on Sn base):
      xi1  : voltage amplitude regulation gain (~0.05-0.2)
      xi2  : reactive pump/damp gain (~0.3-1.0, sign-indefinite)
      mp   : active power-frequency droop (typ. 0.05, i.e. 5% droop)
      omega_lpf : power LPF cutoff (rad/s, typ 20-100 for 3-16 Hz)
    """
    core = DynamicsCore(
        label=f'GFM_{gfm_data["idx"]}',
        dynamics_fn=gfm_inverter_dynamics
    )

    Sn = gfm_data.get('Sn', 100.0)

    metadata = {
        'idx'       : gfm_data['idx'],
        'bus'       : gfm_data['bus'],
        'Sn'        : Sn,
        # LC filter parameters
        'Lf'        : gfm_data.get('Lf', 0.08),
        'Cf'        : gfm_data.get('Cf', 0.074),
        'Rf'        : gfm_data.get('Rf', 0.01),
        # Voltage and frequency references
        'u_ref'     : gfm_data.get('u_ref', 1.03),
        'omega0'    : gfm_data.get('omega0', 1.0),
        # Control gains
        'xi1'       : gfm_data.get('xi1', 0.5),     # voltage convergence gain
        'xi2'       : gfm_data.get('xi2', 2.0),     # reactive pump/damp gain
        'mp'        : gfm_data.get('mp', 0.05),     # P-f droop gain (5%)
        # Power LPF
        'omega_lpf' : gfm_data.get('omega_lpf', 31.4),  # 5 Hz cutoff
        # Power references
        'Pref'      : gfm_data.get('Pref', 0.35),
        'Qref'      : gfm_data.get('Qref', 0.0),
        # Hysteresis / limits
        'hysteresis': gfm_data.get('hysteresis', 0.01),
        'Imax'      : gfm_data.get('Imax', 1.3),
        # CbI passive coupling
        'eps'       : gfm_data.get('eps', 1e-5),
        # Internal hysteresis state
        '_sgn_xi2'  : 1.0,
    }

    # ── Symbolic PH structure (for energy/Lyapunov analysis) ──
    u_sym, th_sym = core.symbols(['u_mag', 'theta'])
    Pf_sym, Qf_sym = core.symbols(['Pf', 'Qf'])
    Cf_sym, eps_sym, ur_sym = core.symbols(['Cf', 'eps', 'u_ref'])

    # H = (Cf/2)*u^2 + (1/2)*((Cf/2)*(u^2-u_ref^2))^2/eps^2
    H_P = (Cf_sym / 2) * u_sym**2
    Phi_sym = (Cf_sym / 2) * (u_sym**2 - ur_sym**2)
    H_C = Phi_sym**2 / (2 * eps_sym**2)
    H   = H_P + H_C

    core.add_storages([u_sym, th_sym, Pf_sym, Qf_sym], H)

    # Dissipation: filter resistance (passive)
    w_u = core.symbols('w_u')
    Rf_sym = core.symbols('Rf'); Lf_sym = core.symbols('Lf')
    core.add_dissipations([w_u], [w_u * Rf_sym / Lf_sym])

    # Ports
    ig_a_sym, ig_b_sym = core.symbols(['ig_a', 'ig_b'])
    u_a_sym, u_b_sym   = core.symbols(['u_a_out', 'u_b_out'])
    core.add_ports([ig_a_sym, ig_b_sym], [u_a_sym, u_b_sym])

    core.subs.update({
        Cf_sym : metadata['Cf'],
        Lf_sym : metadata['Lf'],
        Rf_sym : metadata['Rf'],
        eps_sym: metadata['eps'],
        ur_sym : metadata['u_ref'],
    })

    core.set_metadata(metadata)
    core.n_states = 4
    core.output_fn = gfm_inverter_output
    core.component_type = "renewable"
    core.model_name = "GFM_INVERTER"

    def init_fn(P0=None, Q0=None, V0=None, theta0=0.0, **kwargs):
        """
        Initialise GFM at the power-flow equilibrium.

        At equilibrium:
          u_mag = V0 (from power flow)
          theta = theta0 (bus voltage angle from power flow)
          Pf = P0 (filtered power = reference)
          Qf = Q0
        The voltage reference is updated to match the operating voltage.
        """
        _P0  = P0  if P0  is not None else metadata['Pref']
        _Q0  = Q0  if Q0  is not None else metadata['Qref']
        _V0  = V0  if V0  is not None else metadata['u_ref']
        _th0 = theta0

        # Update references so Pf=Pref and Qf=Qref at t=0
        metadata['Pref']  = _P0
        metadata['Qref']  = _Q0
        metadata['u_ref'] = _V0   # track the actual operating voltage
        metadata['_sgn_xi2'] = 1.0

        return np.array([_V0, _th0, _P0, _Q0])

    core.init_fn = init_fn
    return core, metadata
