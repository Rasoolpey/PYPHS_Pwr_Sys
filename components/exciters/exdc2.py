"""EXDC2 Exciter Model"""
import sys
sys.path.insert(0, '/home/claude')
from utils.pyphs_core import DynamicsCore
import numpy as np


def exdc2_dynamics(x, ports, meta):
    """
    Numerical dynamics for EXDC2 exciter.

    Args:
        x: numpy array of 4 states [vm, vr1, vr2, vf]
           vm: voltage measurement
           vr1: regulator output (Efd)
           vr2: lead-lag state
           vf: feedback state
        ports: dict with keys {'Vt'}
        meta: dict of exciter parameters

    Returns:
        x_dot: numpy array of 4 state derivatives
    """
    # Extract states
    vm, vr1, vr2, vf = x

    # Extract ports
    Vt = ports.get('Vt', 1.0)

    # Extract parameters
    TR = meta['TR']
    TA = meta['TA']
    KA = meta['KA']
    Vref = meta['Vref']
    VRMAX = meta['VRMAX']
    VRMIN = meta['VRMIN']

    # State derivatives
    x_dot = np.zeros(4)

    # Voltage measurement dynamics
    x_dot[0] = (Vt - vm) / TR  # d(vm)/dt

    # Voltage error
    Verr = Vref - vm

    # Regulator dynamics with anti-windup
    vr1_unlimited = (KA * Verr - vr1) / TA

    # Anti-windup limiter
    if vr1 >= VRMAX and vr1_unlimited > 0:
        x_dot[1] = 0.0
    elif vr1 <= VRMIN and vr1_unlimited < 0:
        x_dot[1] = 0.0
    else:
        x_dot[1] = vr1_unlimited

    # vr2, vf not used in simplified model
    x_dot[2] = 0.0
    x_dot[3] = 0.0

    return x_dot


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
    TR = exc_data['TR']
    KA = min(exc_data['KA'], 200.0)
    TA = max(exc_data['TA'], 0.001)
    KE = exc_data['KE']
    TE = max(exc_data['TE'], 0.001)
    KF = exc_data.get('KF1', 0.0754)
    TF1 = max(exc_data['TF1'], 0.001)
    TC = exc_data['TC']
    TB = exc_data['TB']
    
    Vref = 1.0
    
    # States
    vm, vr1, vr2, vf = core.symbols(['vm', 'vr1', 'vr2', 'vf'])
    
    # Parameters
    TR_sym, KA_sym, TA_sym = core.symbols(['TR_exc', 'KA_exc', 'TA_exc'])
    KE_sym, TE_sym = core.symbols(['KE_exc', 'TE_exc'])
    KF_sym, TF1_sym = core.symbols(['KF_exc', 'TF1_exc'])
    TC_sym, TB_sym = core.symbols(['TC_exc', 'TB_exc'])
    Vref_sym = core.symbols('Vref_exc')
    
    # Hamiltonian
    H_exc = (TR_sym/2)*vm**2 + (TA_sym/2)*vr1**2 + (TB_sym/2)*vr2**2 + (TF1_sym/2)*vf**2
    
    core.add_storages([vm, vr1, vr2, vf], H_exc)
    
    # Dissipations
    w_vm = core.symbols('w_vm')
    z_vm = w_vm / TR_sym
    
    w_vr1 = core.symbols('w_vr1')
    z_vr1 = w_vr1 / TA_sym
    
    w_vr2 = core.symbols('w_vr2')
    z_vr2 = w_vr2 / TB_sym
    
    w_vf = core.symbols('w_vf')
    z_vf = w_vf / TF1_sym
    
    core.add_dissipations([w_vm, w_vr1, w_vr2, w_vf],
                         [z_vm, z_vr1, z_vr2, z_vf])
    
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

    return core, metadata
