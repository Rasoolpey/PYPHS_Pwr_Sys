"""TGOV1 Governor Model"""
import sys
sys.path.insert(0, '/home/claude')
from utils.pyphs_core import Core


def build_tgov1_core(gov_data, S_machine=900.0, S_system=100.0):
    """Build TGOV1 governor as PyPHS Core
    
    Args:
        gov_data: dict with governor parameters
        S_machine: machine base power (MVA)
        S_system: system base power (MVA)
    
    Returns:
        core: PyPHS Core object
        metadata: dict with additional info
    """
    core = Core(label=f'TGOV1_{gov_data["idx"]}')
    
    # Extract and scale parameters
    R_machine = max(gov_data['R'], 0.001)
    R_val = R_machine * (S_system / S_machine)
    T1 = max(gov_data['T1'], 0.001)
    T2 = max(gov_data['T2'], 0.001)
    T3 = max(gov_data['T3'], 0.001)
    Dt = gov_data.get('Dt', 0.0)
    
    wref = 1.0
    Pref = 1.0
    
    # States
    x1, x2 = core.symbols(['x1_gov', 'x2_gov'])
    
    # Parameters
    R_sym, T1_sym, T2_sym, T3_sym = core.symbols(['R_gov', 'T1_gov', 'T2_gov', 'T3_gov'])
    Dt_sym = core.symbols('Dt_gov')
    wref_sym, Pref_sym = core.symbols(['wref_gov', 'Pref_gov'])
    
    # Hamiltonian
    H_gov = (T1_sym/2)*x1**2 + (T3_sym/2)*x2**2
    
    core.add_storages([x1, x2], H_gov)
    
    # Dissipations
    w_x1 = core.symbols('w_x1')
    z_x1 = w_x1 / T1_sym
    
    w_x2 = core.symbols('w_x2')
    z_x2 = w_x2 / T3_sym
    
    core.add_dissipations([w_x1, w_x2], [z_x1, z_x2])
    
    # Ports: Input [omega_in], Output [Tm_out]
    omega_in = core.symbols('omega_in')
    Tm_out = core.symbols('Tm_out')
    
    core.add_ports([omega_in], [Tm_out])
    
    # Parameter substitutions
    core.subs.update({
        R_sym: R_val,
        T1_sym: T1,
        T2_sym: T2,
        T3_sym: T3,
        Dt_sym: Dt,
        wref_sym: wref,
        Pref_sym: Pref
    })
    
    # Metadata
    metadata = {
        'idx': gov_data['idx'],
        'syn': gov_data['syn'],
        'R': R_val,
        'T1': T1,
        'T2': T2,
        'T3': T3,
        'VMAX': gov_data['VMAX'],
        'VMIN': gov_data['VMIN'],
        'Dt': Dt,
        'wref': wref,
        'Pref': Pref
    }
    
    return core, metadata
