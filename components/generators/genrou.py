"""GENROU Generator Model"""
import sys
sys.path.insert(0, '/home/claude')
from utils.pyphs_core import Core
import numpy as np


def build_genrou_core(gen_data, S_system=100.0):
    """Build GENROU generator as PyPHS Core
    
    Args:
        gen_data: dict with generator parameters
        S_system: system base power (MVA)
    
    Returns:
        core: PyPHS Core object
        metadata: dict with additional info
    """
    core = Core(label=f'GENROU_{gen_data["idx"]}')
    
    # Extract and scale parameters
    Sn = gen_data.get('Sn', 900.0)
    Z_scale = S_system / Sn
    M_scale = Sn / S_system
    
    M_val = gen_data['M'] * M_scale
    D_base = gen_data.get('D', 0.0)
    D_val = (D_base + 2.0) * M_scale  # Add 2.0 pu damping for stability
    
    ra_val = gen_data['ra'] * Z_scale
    xd = gen_data['xd'] * Z_scale
    xq = gen_data['xq'] * Z_scale
    xd1 = gen_data['xd1'] * Z_scale
    xq1 = gen_data['xq1'] * Z_scale
    xd2 = gen_data['xd2'] * Z_scale
    xq2 = gen_data['xq2'] * Z_scale
    xl = gen_data['xl'] * Z_scale
    
    Td10 = gen_data['Td10']
    Td20 = gen_data['Td20']
    Tq10 = gen_data['Tq10']
    Tq20 = gen_data['Tq20']
    
    omega_b = 2 * np.pi * gen_data.get('fn', 60.0)
    
    # Derive inductances
    Xad = xd - xl
    Xaq = xq - xl
    
    Xfl = (Xad * (xd1 - xl)) / (Xad - (xd1 - xl))
    Lf_val = Xad + Xfl
    
    Xkql = (Xaq * (xq1 - xl)) / (Xaq - (xq1 - xl))
    Lkq_val = Xaq + Xkql
    
    Xkdl = max(0.05, xd2 - xl)
    Lkd_val = Xad + Xkdl
    
    Ld2_val = xd2
    Lq2_val = xq2
    
    # GENROU coefficients
    gd1 = (xd2 - xl) / (xd1 - xl)
    gd2 = (xd1 - xd2) / ((xd1 - xl)**2)
    gq1 = (xq2 - xl) / (xq1 - xl)
    gq2 = (xq1 - xq2) / ((xq1 - xl)**2)
    
    # States: [delta, p, psi_d, psi_q, psi_f, psi_kd, psi_kq]
    delta, p = core.symbols(['delta', 'p'])
    psi_d, psi_q, psi_f, psi_kd, psi_kq = core.symbols(['psi_d', 'psi_q', 'psi_f', 'psi_kd', 'psi_kq'])
    
    # Parameters
    M_sym, D_sym, omega_b_sym = core.symbols(['M', 'D', 'omega_b'])
    Ld2, Lq2 = core.symbols(['Ld2', 'Lq2'])
    Lf, Lkd, Lkq = core.symbols(['Lf', 'Lkd', 'Lkq'])
    Td10_sym, Td20_sym, Tq20_sym = core.symbols(['Td10', 'Td20', 'Tq20'])
    ra = core.symbols('ra')
    
    # Hamiltonian
    H_mech = p**2 / (2*M_sym)
    H_elec = (psi_d**2/(2*Ld2) + psi_q**2/(2*Lq2) + 
              psi_f**2/(2*Lf) + psi_kd**2/(2*Lkd) + psi_kq**2/(2*Lkq))
    H_total = H_mech + H_elec
    
    core.add_storages([delta, p, psi_d, psi_q, psi_f, psi_kd, psi_kq], H_total)
    
    # Dissipations
    w_damp = core.symbols('w_damp')
    z_damp = D_sym * w_damp
    
    w_ra_d, w_ra_q = core.symbols(['w_ra_d', 'w_ra_q'])
    z_ra_d = ra * w_ra_d
    z_ra_q = ra * w_ra_q
    
    w_f = core.symbols('w_f')
    z_f = w_f / Td10_sym
    
    w_kd, w_kq = core.symbols(['w_kd', 'w_kq'])
    z_kd = w_kd / Td20_sym
    z_kq = w_kq / Tq20_sym
    
    core.add_dissipations([w_damp, w_ra_d, w_ra_q, w_f, w_kd, w_kq],
                         [z_damp, z_ra_d, z_ra_q, z_f, z_kd, z_kq])
    
    # Ports: Inputs [Tm, Efd, Vd_net, Vq_net], Outputs [Id_out, Iq_out]
    Tm, Efd = core.symbols(['Tm', 'Efd'])
    Vd_net, Vq_net = core.symbols(['Vd_net', 'Vq_net'])
    Id_out, Iq_out = core.symbols(['Id_out', 'Iq_out'])
    
    core.add_ports([Tm, Efd, Vd_net, Vq_net], [0, 0, Id_out, Iq_out])
    
    # Parameter substitutions
    core.subs.update({
        M_sym: M_val,
        D_sym: D_val,
        omega_b_sym: omega_b,
        Ld2: Ld2_val,
        Lq2: Lq2_val,
        Lf: Lf_val,
        Lkd: Lkd_val,
        Lkq: Lkq_val,
        Td10_sym: Td10,
        Td20_sym: Td20,
        Tq20_sym: Tq20,
        ra: ra_val
    })
    
    # Metadata
    metadata = {
        'idx': gen_data['idx'],
        'bus': gen_data['bus'],
        'Sn': Sn,
        'omega_b': omega_b,
        'xd': xd, 'xq': xq,
        'xd1': xd1, 'xq1': xq1,
        'xd2': xd2, 'xq2': xq2,
        'xl': xl,
        'gd1': gd1, 'gd2': gd2,
        'gq1': gq1, 'gq2': gq2,
        'ra': ra_val,
        'M': M_val,
        'D': D_val
    }
    
    return core, metadata
