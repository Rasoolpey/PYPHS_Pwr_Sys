"""ESST3A Exciter Model - IEEE ST3A Static Excitation System"""
import sys
sys.path.insert(0, '/home/claude')
from utils.pyphs_core import DynamicsCore
import numpy as np


def esst3a_dynamics(x, ports, meta):
    """
    Numerical dynamics for ESST3A exciter.
    
    Full IEEE ESST3A model with voltage compensation and rectifier
    
    Args:
        x: numpy array of 5 states [LG_y, LL_exc_x, VR, VM, VB_state]
           LG_y: Voltage transducer lag output
           LL_exc_x: Lead-lag compensator state
           VR: Voltage regulator output
           VM: Inner field voltage regulator output
           VB_state: Rectifier voltage state (for continuity)
        ports: dict with keys {'Vt', 'Id', 'Iq', 'Vd', 'Vq', 'Efd_fb'}
        meta: dict of exciter parameters
    
    Returns:
        x_dot: numpy array of 5 state derivatives
    """
    # Extract states
    LG_y, LL_exc_x, VR, VM, VB_state = x
    
    # Extract ports
    Vt = ports.get('Vt', 1.0)
    Id = ports.get('Id', 0.0)
    Iq = ports.get('Iq', 0.0)
    Vd = ports.get('Vd', 0.0)
    Vq = ports.get('Vq', 0.0)
    XadIfd = ports.get('XadIfd', 1.0)  # Field current equivalent
    
    # Extract parameters
    TR = meta['TR']
    KA = meta['KA']
    TA = meta['TA']
    TC = meta['TC']
    TB = meta['TB']
    KM = meta['KM']
    TM = meta['TM']
    VRMAX = meta['VRMAX']
    VRMIN = meta['VRMIN']
    VMMAX = meta['VMMAX']
    VMMIN = meta['VMMIN']
    VIMAX = meta['VIMAX']
    VIMIN = meta['VIMIN']
    VBMAX = meta['VBMAX']
    KC = meta['KC']
    KP = meta['KP']
    KI = meta['KI']
    XL = meta['XL']
    KG = meta['KG']
    VGMAX = meta['VGMAX']
    THETAP = meta['THETAP']
    # Use vref computed by init_fn (includes voltage error offset for equilibrium)
    vref = meta.get('vref', 1.0)
    
    # State derivatives
    x_dot = np.zeros(5)
    
    # 1. Voltage transducer (TR lag)
    if TR > 1e-6:
        x_dot[0] = (Vt - LG_y) / TR
    else:
        x_dot[0] = 0.0
    
    # 2. Voltage error
    vi = vref - LG_y
    
    # 3. Input limiter
    vil = np.clip(vi, VIMIN, VIMAX)
    
    # 4. Lead-lag compensator (TC/TB)
    if TC > 1e-6:
        LL_exc_y = (TB / TC) * (vil - LL_exc_x) + LL_exc_x
        x_dot[1] = (vil - LL_exc_x) / TC
    else:
        LL_exc_y = vil
        x_dot[1] = 0.0
    
    # 5. Voltage regulator with anti-windup (KA, TA)
    VR_unlimited = KA * LL_exc_y
    
    if TA > 1e-6:
        VR_limited = np.clip(VR_unlimited, VRMIN, VRMAX)
        VR_derivative = (VR_limited - VR) / TA
        
        # Anti-windup: prevent integration if at limit and trying to go further
        if VR >= VRMAX and VR_derivative > 0:
            x_dot[2] = 0.0
        elif VR <= VRMIN and VR_derivative < 0:
            x_dot[2] = 0.0
        else:
            x_dot[2] = VR_derivative
    else:
        x_dot[2] = 0.0
    
    # 6. Compute VE (sensed excitation voltage with compensation)
    # VE = |KPC*(Vd + j*Vq) + j*(KI + KPC*XL)*(Id + j*Iq)|
    KPC = KP * np.exp(1j * np.radians(THETAP))
    z1 = KPC * (Vd + 1j * Vq)
    z2 = 1j * (KI + KPC * XL) * (Id + 1j * Iq)
    VE = np.abs(z1 + z2)
    
    # 7. Rectifier regulation
    if VE > 1e-6:
        IN = KC * XadIfd / VE
    else:
        IN = 0.0
    
    # 8. Rectifier loading factor FEX (piecewise function)
    if IN <= 0:
        FEX = 1.0
    elif IN <= 0.433:
        FEX = 1.0 - 0.577 * IN
    elif IN <= 0.75:
        FEX = np.sqrt(0.75 - IN**2)
    elif IN <= 1.0:
        FEX = 1.732 * (1.0 - IN)
    else:
        FEX = 0.0
    
    # 9. VB calculation with limiter
    VB = VE * FEX
    VB = np.clip(VB, 0.0, VBMAX)  # Realistic bounds
    
    # VB state for smooth dynamics (first-order lag) with anti-windup
    # Limit VB_state to prevent unrealistic field voltages
    # Typical exciter ceiling is 5 pu, so VB_state should not exceed this
    VB_state_max = min(VBMAX, 5.0)
    VB_derivative = (VB - VB_state) / 0.01
    
    if VB_state >= VB_state_max and VB_derivative > 0:
        x_dot[4] = 0.0  # Stop increasing at limit
    elif VB_state <= 0.0 and VB_derivative < 0:
        x_dot[4] = 0.0  # Don't go negative
    else:
        x_dot[4] = VB_derivative
    
    # 10. Feedback path VG = KG * Efd
    # Apply the same limit to Efd calculation
    VB_state_limited = np.clip(VB_state, 0.0, VB_state_max)
    VM_limited_internal = np.clip(VM, 0.1, 10.0)  # Also limit VM for Efd calculation
    Efd = VB_state_limited * VM_limited_internal
    Efd = np.clip(Efd, 0.0, 5.0)  # Final hard limit on Efd output
    VG = KG * Efd
    VG = np.clip(VG, 0.0, VGMAX)  # Also limit VG lower bound
    
    # 11. VRS calculation
    vrs = VR - VG
    
    # 12. Inner field regulator with anti-windup (KM, TM)
    VM_unlimited = KM * vrs
    
    # Apply realistic limits to VM (typically 1-10 for power system exciters)
    # Override VMMAX if it's unrealistically high
    VMMAX_realistic = min(VMMAX, 10.0) if VMMAX < 90 else 10.0
    
    if TM > 1e-6:
        VM_limited = np.clip(VM_unlimited, VMMIN, VMMAX_realistic)
        VM_derivative = (VM_limited - VM) / TM
        
        # Anti-windup: prevent integration if at limit and trying to go further
        if VM >= VMMAX_realistic and VM_derivative > 0:
            x_dot[3] = 0.0
        elif VM <= VMMIN and VM_derivative < 0:
            x_dot[3] = 0.0
        else:
            x_dot[3] = VM_derivative
    else:
        x_dot[3] = 0.0
    
    return x_dot


def esst3a_output(x, ports, meta):
    """
    Compute exciter output Efd.
    
    Args:
        x: numpy array of states
        ports: dict with voltage and current info
        meta: dict of parameters
    
    Returns:
        Efd: field voltage (limited to realistic values)
    """
    LG_y, LL_exc_x, VR, VM, VB_state = x
    
    # Efd = VB * VM
    Efd_unlimited = VB_state * VM
    
    # Apply realistic field voltage limits (typically 5-8 pu for power system exciters)
    # This prevents unrealistic excitation levels
    Efd_max = meta.get('Efd_max', 5.0)  # Default 5 pu limit
    Efd_min = meta.get('Efd_min', 0.0)
    Efd = np.clip(Efd_unlimited, Efd_min, Efd_max)
    
    return Efd


def build_esst3a_core(exc_data):
    """Build ESST3A exciter as DynamicsCore
    
    Args:
        exc_data: dict with exciter parameters
    
    Returns:
        core: DynamicsCore object with dynamics method
        metadata: dict with additional info
    """
    core = DynamicsCore(label=f'ESST3A_{exc_data["idx"]}', dynamics_fn=esst3a_dynamics)
    
    # Extract parameters
    TR = exc_data.get('TR', 0.02)
    KA = exc_data.get('KA', 20.0)
    TA = exc_data.get('TA', 0.02)
    TC = exc_data.get('TC', 1.0)
    TB = exc_data.get('TB', 5.0)
    KM = exc_data.get('KM', 8.0)
    TM = exc_data.get('TM', 0.4)
    VRMAX = exc_data.get('VRMAX', 99.0)
    VRMIN = exc_data.get('VRMIN', -99.0)
    VMMAX = exc_data.get('VMMAX', 99.0)
    VMMIN = exc_data.get('VMMIN', 0.0)
    VIMAX = exc_data.get('VIMAX', 0.2)
    VIMIN = exc_data.get('VIMIN', -0.2)
    VBMAX = exc_data.get('VBMAX', 5.48)
    KC = exc_data.get('KC', 0.01)
    KP = exc_data.get('KP', 3.67)
    KI = exc_data.get('KI', 0.435)
    XL = exc_data.get('XL', 0.0098)
    KG = exc_data.get('KG', 1.0)
    VGMAX = exc_data.get('VGMAX', 3.86)
    THETAP = exc_data.get('THETAP', 3.33)
    
    # States
    LG_y, LL_exc_x, VR, VM, VB_state = core.symbols([
        'LG_y_exc', 'LL_exc_x', 'VR_exc', 'VM_exc', 'VB_exc'
    ])
    
    # Parameters (symbolic)
    TR_sym, TA_sym, TC_sym, TM_sym = core.symbols(['TR', 'TA', 'TC', 'TM'])
    
    # Hamiltonian (energy storage in lag blocks)
    H_exc = (TR_sym/2) * LG_y**2 + (TC_sym/2) * LL_exc_x**2 + \
            (TA_sym/2) * VR**2 + (TM_sym/2) * VM**2 + 0.5 * VB_state**2
    
    core.add_storages([LG_y, LL_exc_x, VR, VM, VB_state], H_exc)
    
    # Dissipations
    w_exc = core.symbols('w_exc')
    z_exc = w_exc
    core.add_dissipations(w_exc, z_exc)
    
    # Ports: Input [Vt], Output [Efd]
    Vt_in = core.symbols('Vt_in')
    Efd_out = core.symbols('Efd_out')
    
    core.add_ports([Vt_in], [Efd_out])
    
    # Parameter substitutions
    core.subs.update({
        TR_sym: TR,
        TA_sym: TA,
        TC_sym: TC,
        TM_sym: TM,
    })
    
    # Metadata
    metadata = {
        'idx': exc_data['idx'],
        'syn': exc_data['syn'],
        'TR': TR,
        'KA': KA,
        'TA': TA,
        'TC': TC,
        'TB': TB,
        'KM': KM,
        'TM': TM,
        'VRMAX': VRMAX,
        'VRMIN': VRMIN,
        'VMMAX': VMMAX,
        'VMMIN': VMMIN,
        'VIMAX': VIMAX,
        'VIMIN': VIMIN,
        'VBMAX': VBMAX,
        'KC': KC,
        'KP': KP,
        'KI': KI,
        'XL': XL,
        'KG': KG,
        'VGMAX': VGMAX,
        'THETAP': THETAP,
        'vref': 1.0,  # Will be updated during initialization
        'Efd_max': exc_data.get('Efd_max', 5.0),  # Realistic exciter limit
        'Efd_min': exc_data.get('Efd_min', 0.0)
    }
    
    # Set metadata on core
    core.set_metadata(metadata)
    
    # Set component interface attributes
    core.n_states = 5
    core.output_fn = esst3a_output
    # Proper initialization function that computes all states correctly
    core.init_fn = lambda Efd_eq, V_mag, **kwargs: esst3a_initialize(Efd_eq, V_mag, metadata, **kwargs)
    core.component_type = "exciter"
    core.model_name = "ESST3A"
    
    return core, metadata


def esst3a_initialize(Efd_target, Vt, metadata, Vd=0.0, Vq=None, Id=0.0, Iq=0.0, psi_f=1.0):
    """
    Compute equilibrium initial states for ESST3A exciter.
    
    At equilibrium, all derivatives = 0:
        LG_y = Vt (voltage measurement converged)
        VB_state = VE * FEX (rectifier converged)
        VM = Efd / VB (multiplier output)
        VR = VM * KM + KG * Efd (regulator chain)
        LL_exc_x = VR / KA (lead-lag input)
        Vref = Vt + LL_exc_x (clamped to VIMAX/VIMIN)
    
    Args:
        Efd_target: Desired field voltage
        Vt: Terminal voltage magnitude
        metadata: ESST3A parameters
        Vd, Vq, Id, Iq: Network quantities for VE calculation
        psi_f: Generator field flux
        
    Returns:
        x0: numpy array [LG_y, LL_exc_x, VR, VM, VB_state]
    """
    # If Vq not provided, assume Vt is the magnitude
    if Vq is None:
        Vq = Vt
        Vd = 0.0
    
    # Extract parameters
    KM = metadata.get('KM', 8.0)
    KG = metadata.get('KG', 1.0)
    KA = metadata.get('KA', 20.0)
    KP = metadata.get('KP', 3.67)
    KI_exc = metadata.get('KI', 0.435)
    XL_exc = metadata.get('XL', 0.0098)
    KC = metadata.get('KC', 0.01)
    THETAP = metadata.get('THETAP', 0.0)
    VBMAX = metadata.get('VBMAX', 5.48)
    VIMAX = metadata.get('VIMAX', 0.2)
    VIMIN = metadata.get('VIMIN', -0.2)
    
    # Compute VE from terminal voltage/current (load compensated)
    KPC = KP * np.exp(1j * np.radians(THETAP))
    z1 = KPC * (Vd + 1j * Vq)
    z2 = 1j * (KI_exc + KPC * XL_exc) * (Id + 1j * Iq)
    VE = np.abs(z1 + z2)
    
    # Rectifier regulation: FEX(IN) where IN = KC * XadIfd / VE
    XadIfd = psi_f
    IN = KC * XadIfd / VE if VE > 1e-6 else 0.0
    
    # FEX function (IEEE ESST3A standard)
    if IN <= 0:
        FEX = 1.0
    elif IN <= 0.433:
        FEX = 1.0 - 0.577 * IN
    elif IN <= 0.75:
        FEX = np.sqrt(0.75 - IN**2)
    elif IN < 1.0:
        FEX = 1.732 * (1.0 - IN)
    else:
        FEX = 0.0
    
    # Rectifier voltage
    VB_eq = np.clip(VE * FEX, 0.0, VBMAX)
    
    # Extract ALL limits to match dynamics function exactly
    VRMAX = metadata.get('VRMAX', 99.0)
    VRMIN = metadata.get('VRMIN', -99.0)
    VMMAX = metadata.get('VMMAX', 99.0)
    VMMIN = metadata.get('VMMIN', 0.0)
    KM = metadata.get('KM', 8.0)
    KG = metadata.get('KG', 1.0)
    VGMAX = metadata.get('VGMAX', 3.86)
    Efd_max = metadata.get('Efd_max', 5.0)
    Efd_min = metadata.get('Efd_min', 0.0)

    # Apply the same limits as the dynamics function
    VMMAX_realistic = min(VMMAX, 10.0) if VMMAX < 90 else 10.0
    VB_state_max = min(VBMAX, 5.0)
    VB_eq_limited = np.clip(VB_eq, 0.0, VB_state_max)

    # Apply hard Efd output limit (same as esst3a_output)
    Efd_eff = np.clip(Efd_target, Efd_min, Efd_max)

    # Trace backwards from Efd through control chain
    VM_eq = Efd_eff / VB_eq_limited if VB_eq_limited > 1e-6 else 1.0
    # Apply internal VM limit for Efd/VG computation (dynamics line 148)
    VM_eq_internal = np.clip(VM_eq, 0.1, 10.0)
    # Recompute effective Efd with internal VM limit
    Efd_eff = np.clip(VB_eq_limited * VM_eq_internal, 0.0, 5.0)  # Hard limit (dynamics line 150)

    # VG with VGMAX limit (dynamics line 152)
    VG_eff = np.clip(KG * Efd_eff, 0.0, VGMAX)

    # Backward through inner regulator: VM = KM * (VR - VG), so vrs = VM/KM
    vrs_eq = VM_eq / KM if KM > 1e-6 else 0.0
    VR_eq = vrs_eq + VG_eff  # Use VG_eff (clipped), not KG*Efd (unclipped)

    vil_eq = VR_eq / KA if KA > 1e-6 else 0.0

    # Apply input limiter - at equilibrium LL_exc_x must equal the
    # CLAMPED input, otherwise x_dot[1] = (vil - LL_exc_x)/TC != 0
    vil_clamped = np.clip(vil_eq, VIMIN, VIMAX)

    if abs(vil_clamped - vil_eq) > 1e-8:
        # Input limiter is active: recompute forward chain
        VR_eq = np.clip(KA * vil_clamped, VRMIN, VRMAX)

        # Solve feedback loop with VGMAX constraint:
        # At equilibrium: Efd = VB * VM, VG = clip(KG*Efd, 0, VGMAX)
        # vrs = VR - VG, VM = KM * vrs
        # If VG < VGMAX (not clipped): Efd = VB*KM*(VR - KG*Efd)
        # If VG = VGMAX (clipped): Efd = VB*KM*(VR - VGMAX)

        # Try unclipped VG first
        denom = 1.0 + VB_eq_limited * KM * KG
        if abs(denom) > 1e-6:
            Efd_try = VB_eq_limited * KM * VR_eq / denom
        else:
            Efd_try = Efd_target
        VG_try = KG * Efd_try

        if VG_try > VGMAX:
            # VG is clipped: use VG = VGMAX
            vrs_eq = VR_eq - VGMAX
            VM_eq = np.clip(KM * vrs_eq, VMMIN, VMMAX_realistic)
            Efd_eff = np.clip(VB_eq_limited * np.clip(VM_eq, 0.1, 10.0), 0.0, 5.0)
        else:
            Efd_eff = np.clip(Efd_try, 0.0, 5.0)
            VG_eff = np.clip(KG * Efd_eff, 0.0, VGMAX)
            vrs_eq = VR_eq - VG_eff
            VM_eq = np.clip(KM * vrs_eq, VMMIN, VMMAX_realistic)

        VM_eq = Efd_eff / VB_eq_limited if VB_eq_limited > 1e-6 else 1.0
    else:
        # No input limiting: apply remaining limits for consistency
        VR_eq = np.clip(VR_eq, VRMIN, VRMAX)
        VM_eq = np.clip(VM_eq, VMMIN, VMMAX_realistic)

    LL_exc_x_eq = vil_clamped

    # Voltage reference: at equilibrium vi = vref - Vt, and vil = clip(vi, VIMIN, VIMAX)
    # We need vil = vil_clamped, so vi must produce that after clipping
    Vref_eq = Vt + vil_clamped

    # Update metadata with equilibrium Vref
    metadata['vref'] = Vref_eq

    # Return initial states: [LG_y, LL_exc_x, VR, VM, VB_state]
    return np.array([Vt, LL_exc_x_eq, VR_eq, VM_eq, VB_eq])

