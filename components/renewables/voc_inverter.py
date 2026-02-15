"""
VOC_INVERTER - Virtual Oscillator Control (VOC) Grid-Forming Inverter
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
    dtheta/dt = omega0 + mp*(Pref - Pf)          (P-f droop, VIRTUAL INERTIA)
    du/dt = xi1*(u_ref^2 - u^2)*u + xi2_signed*(Qref/u_ref^2 - Qf/u^2)*u
            (Q-u oscillator amplitude with pumping-or-damping)

Virtual Inertia:
    The P-f droop (mp parameter) provides frequency response similar to synchronous machines
    Typical mp = 0.05 (5% droop) means 0.05 pu freq deviation per 1 pu power mismatch
    Equivalent H_virtual ≈ (1 / (2 * pi * f0 * mp)) ≈ 3.2s for mp=0.05

Parameters (JSON):
  Sn, bus        : rating and bus number
  Lf, Cf, Rf    : LC filter (pu)
  u_ref          : voltage reference magnitude (pu)
  omega0         : synchronous frequency reference (pu = 1.0 for 60 Hz)
  xi1            : voltage convergence gain (amplitude regulation, typ. 0.05-0.2)
  xi2            : reactive power pump/damp gain (sign-indefinite, typ. 0.3-1.0)
  mp             : active power - frequency droop gain (pu/pu, typ. 0.05 for 5% droop)
  omega_lpf      : LPF cutoff for P/Q measurement (rad/s in pu time, typ. 20-100)
  Pref, Qref     : power references (pu)
  hysteresis     : hysteresis band for pump/damp switching
  Imax           : current limit (pu)
"""

import numpy as np
from numba import njit


@njit(cache=True)
def voc_inverter_dynamics_jit(x, ig_a, ig_b, Cf, u_ref, omega0, xi1, xi2_abs,
                               mp, omega_lpf, Pref, Qref, hysteresis, _sgn_xi2_prev):
    """
    JIT-compiled VOC inverter dynamics - polar voltage representation.
    
    States:
      x[0] = u_mag  : |u(t)| voltage magnitude (pu)
      x[1] = theta  : virtual oscillator angle (rad)
      x[2] = Pf     : filtered active power (pu)
      x[3] = Qf     : filtered reactive power (pu)
    
    Returns:
      (x_dot, sgn_xi2_new) where x_dot is numpy array shape (4,)
    """
    u_mag = x[0]
    theta = x[1]
    Pf = x[2]
    Qf = x[3]
    
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
    u2 = u_mag * u_mag
    u2_ref = u_ref * u_ref
    eps = 1e-5  # Small value for numerical stability
    Phi = 0.5 * Cf * (u2 - u2_ref)  # storage variable, zero at u=u_ref
    
    sgn_xi2 = _sgn_xi2_prev  # default: keep previous sign
    Q_ratio = 0.0
    
    if u2 > 1e-6:
        Q_ratio = Qref / u2_ref - Qf / u2
        # Passivity condition (eq.18): sgn(xi2) must oppose sgn(Q_ratio * Phi)
        signal = Q_ratio * Phi
        if signal > hysteresis:
            sgn_xi2 = -1.0
        elif signal < -hysteresis:
            sgn_xi2 = 1.0
        # else: keep previous sign (hysteresis band)
        
    xi2 = sgn_xi2 * xi2_abs
    
    # ---------- Voltage amplitude dynamics (eq. 10 and u-dot equation) ----------
    # From eq.(19) with the outer loop only:
    #   du/dt = A/Cf * u   (in magnitude, since rotation is handled by theta)
    # where A = Cf*xi1*(u_ref^2 - u^2) + Cf*xi2*(Qref/u_ref^2 - Qf/u^2)
    A = Cf * xi1 * (u2_ref - u2) + Cf * xi2 * Q_ratio
    du_mag = (A / Cf) * u_mag  # magnitude: du/dt = A/Cf * |u|
    
    # ---------- Phase angle / frequency (eq. 11) - VIRTUAL INERTIA ----------
    # This P-f droop provides frequency response similar to synchronous inertia
    # dθ/dt = omega0 + mp*(Pref - Pf)
    # At equilibrium Pf=Pref so dθ/dt = omega0 (steady rotation)
    # When power deviates, frequency deviates proportionally (droop characteristic)
    if u2 > 1e-6:
        omega = omega0 + mp * (Pref / u2_ref - Pf / u2) * u2_ref
    else:
        omega = omega0
    dtheta = omega
    
    x_dot = np.array([du_mag, dtheta, dPf, dQf])
    return x_dot, sgn_xi2


def voc_inverter_dynamics(x, ports, meta):
    """
    VOC inverter dynamics - wrapper for JIT function.
    
    States:
      x[0] = u_mag  : |u(t)| voltage magnitude (pu)
      x[1] = theta  : virtual oscillator angle (rad)
      x[2] = Pf     : filtered active power (pu)
      x[3] = Qf     : filtered reactive power (pu)
    
    Ports:
      'ig_a', 'ig_b' : grid current components (pu, αβ frame)
    
    Returns:
      numpy array shape (4,)
    """
    # Current injected into the bus from external network
    ig_a = ports.get('ig_a', 0.0)
    ig_b = ports.get('ig_b', 0.0)
    
    # Call JIT function
    x_dot, sgn_xi2_new = voc_inverter_dynamics_jit(
        x, ig_a, ig_b,
        meta['Cf'], meta['u_ref'], meta['omega0'],
        meta['xi1'], meta['xi2_abs'], meta['mp'],
        meta['omega_lpf'], meta['Pref'], meta['Qref'],
        meta['hysteresis'], meta['_sgn_xi2']
    )
    
    # Update sign state in metadata
    meta['_sgn_xi2'] = sgn_xi2_new
    
    return x_dot


@njit(cache=True)
def voc_inverter_output_jit(x, ig_a, ig_b, u_ref, omega0, mp, Pref):
    """JIT-compiled output function."""
    u_mag = x[0]
    theta = x[1]
    Pf = x[2]
    Qf = x[3]
    
    u_a = u_mag * np.cos(theta)
    u_b = u_mag * np.sin(theta)
    Pe = u_a * ig_a + u_b * ig_b
    Qe = u_b * ig_a - u_a * ig_b
    
    u2 = u_mag * u_mag
    u2_ref = u_ref * u_ref
    if u2 > 1e-6:
        omega = omega0 + mp * (Pref / u2_ref - Pf / u2) * u2_ref
    else:
        omega = omega0
    
    # Current outputs (for network interface compatibility)
    if u_mag > 0.01:
        Ipout = Pe / u_mag
        Iqout = Qe / u_mag
    else:
        Ipout = 0.0
        Iqout = 0.0
    
    return Pe, Qe, u_a, u_b, u_mag, theta, omega, Ipout, Iqout


def voc_inverter_output(x, ports, meta):
    """
    Compute VOC inverter outputs for network interface.
    
    Returns:
      dict: Pe, Qe, u_a, u_b (αβ terminal voltage), V, theta, omega, Ipout, Iqout
    """
    ig_a = ports.get('ig_a', 0.0)
    ig_b = ports.get('ig_b', 0.0)
    
    Pe, Qe, u_a, u_b, V, theta, omega, Ipout, Iqout = voc_inverter_output_jit(
        x, ig_a, ig_b, meta['u_ref'], meta['omega0'], meta['mp'], meta['Pref']
    )
    
    return {
        'Pe': Pe,
        'Qe': Qe,
        'u_a': u_a,
        'u_b': u_b,
        'V': V,
        'theta': theta,
        'omega': omega,
        'Ipout': Ipout,
        'Iqout': Iqout,
    }


def pack_voc_meta(voc_data):
    """Pack VOC metadata for JIT compatibility."""
    return {
        'idx': voc_data['idx'],
        'bus': voc_data['bus'],
        'Sn': voc_data.get('Sn', 100.0),
        # LC filter parameters
        'Lf': voc_data.get('Lf', 0.08),
        'Cf': voc_data.get('Cf', 0.074),
        'Rf': voc_data.get('Rf', 0.01),
        # Voltage and frequency references
        'u_ref': voc_data.get('u_ref', 1.03),
        'omega0': voc_data.get('omega0', 1.0),
        # Control gains
        'xi1': voc_data.get('xi1', 0.06),  # voltage convergence gain
        'xi2_abs': abs(voc_data.get('xi2', 0.42)),  # reactive pump/damp gain (absolute value)
        'mp': voc_data.get('mp', 0.05),  # P-f droop gain (5% droop = ~3.2s virtual inertia)
        # Power LPF
        'omega_lpf': voc_data.get('omega_lpf', 62.83),  # 10 Hz cutoff (2*pi*10)
        # Power references
        'Pref': voc_data.get('Pref', 0.35),
        'Qref': voc_data.get('Qref', 0.0),
        # Hysteresis / limits
        'hysteresis': voc_data.get('hysteresis', 0.01),
        'Imax': voc_data.get('Imax', 1.3),
        # CbI passive coupling
        'eps': voc_data.get('eps', 1e-5),
        # Internal hysteresis state
        '_sgn_xi2': 1.0,
    }


def voc_init(P0=None, Q0=None, V0=None, theta0=0.0, meta=None, **kwargs):
    """
    Initialize VOC inverter at the power-flow equilibrium.
    
    At equilibrium:
      u_mag = V0 (from power flow)
      theta = theta0 (bus voltage angle from power flow)
      Pf = P0 (filtered power = reference)
      Qf = Q0
    
    The voltage reference is updated to match the operating voltage.
    """
    _P0 = P0 if P0 is not None else meta['Pref']
    _Q0 = Q0 if Q0 is not None else meta['Qref']
    _V0 = V0 if V0 is not None else meta['u_ref']
    _th0 = theta0
    
    # Update references so Pf=Pref and Qf=Qref at t=0
    meta['Pref'] = _P0
    meta['Qref'] = _Q0
    meta['u_ref'] = _V0  # track the actual operating voltage
    meta['_sgn_xi2'] = 1.0
    
    return np.array([_V0, _th0, _P0, _Q0])


def build_voc_inverter_core(voc_data, S_system=100.0):
    """
    Build VOC inverter component for integration with fault_sim_modular.
    This function follows the naming convention required by ComponentFactory.
    
    Args:
        voc_data: dict with VOC parameters from JSON
        S_system: system base power (MVA)
    
    Returns:
        (None, metadata) tuple - None for core since VOC doesn't use DynamicsCore,
        metadata dict with all component info
    """
    metadata = pack_voc_meta(voc_data)
    
    # Add component description fields
    metadata['n_states'] = 4
    metadata['model_name'] = 'VOC_INVERTER'
    metadata['component_type'] = 'renewable'
    
    # Add function references for dynamics integration
    metadata['dynamics_fn'] = voc_inverter_dynamics
    metadata['output_fn'] = voc_inverter_output
    metadata['init_fn'] = voc_init
    
    # Virtual inertia calculation for documentation
    # H_virtual ≈ 1 / (2 * pi * f0 * mp) in seconds
    # For mp = 0.05, H_virtual ≈ 3.18s (comparable to small synchronous machine)
    mp_val = metadata['mp']
    f0 = 60.0  # Hz
    H_virtual = 1.0 / (2 * np.pi * f0 * mp_val) if mp_val > 0 else 0.0
    metadata['H_virtual'] = H_virtual
    metadata['droop_percentage'] = mp_val * 100  # percent
    
    # System base for per-unit conversions
    metadata['S_system'] = S_system
    
    return None, metadata  # Return None for core (not using DynamicsCore object)
