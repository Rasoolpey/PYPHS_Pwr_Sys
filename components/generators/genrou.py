"""GENROU Generator Model"""
import sys
sys.path.insert(0, '/home/claude')
from utils.pyphs_core import DynamicsCore
import numpy as np


def genrou_dynamics(x, ports, meta):
    """
    Numerical dynamics for GENROU generator.

    Args:
        x: numpy array of 7 states [delta, p, psi_d, psi_q, psi_f, psi_kd, psi_kq]
        ports: dict with keys {'Id', 'Iq', 'Vd', 'Vq', 'Tm', 'Efd'}
        meta: dict of generator parameters

    Returns:
        x_dot: numpy array of 7 state derivatives
    """
    # Extract states
    delta, p, psi_d, psi_q, psi_f, psi_kd, psi_kq = x

    # Extract ports
    Id = ports.get('Id', 0.0)
    Iq = ports.get('Iq', 0.0)
    Vd = ports.get('Vd', 0.0)
    Vq = ports.get('Vq', 0.0)
    Tm = ports.get('Tm', 0.0)
    Efd = ports.get('Efd', 1.0)

    # Extract parameters
    M = meta['M']
    D = meta['D']
    ra = meta['ra']
    omega_b = meta['omega_b']
    Td10 = meta['Td10']
    Td20 = meta['Td20']
    Tq20 = meta['Tq20']
    
    # Get field inductance scaling factor for proper Efd per-unit base
    # Efd from exciter is on field voltage base (≈1 pu at rated conditions)
    # psi_f is field flux on stator base (can be 5-6 pu at rated conditions)
    # The scaling factor converts between these bases
    xd = meta.get('xd', 1.8)
    xl = meta.get('xl', 0.15)
    xd1 = meta.get('xd1', 0.6)
    Xad = xd - xl
    
    # Calculate field inductance Lf
    Xfl = (Xad * (xd1 - xl)) / (Xad - (xd1 - xl))
    Lf = Xad + Xfl
    
    # Scaling factor: Use Lf*2 for typical exciter per-unit base
    # This gives Efd ≈ 1.2-1.5 pu at rated conditions
    Kfd_scale = Lf * 2.0

    # Rotor speed from momentum
    omega = p / M

    # Electrical torque
    Te = Vd * Id + Vq * Iq

    # State derivatives
    x_dot = np.zeros(7)

    # Swing equations
    x_dot[0] = omega_b * (omega - 1.0)  # d(delta)/dt
    x_dot[1] = Tm - Te - D * (omega - 1.0)  # d(p)/dt = M * d(omega)/dt

    # Stator flux linkage equations
    x_dot[2] = Vd - ra * Id + omega_b * omega * psi_q  # d(psi_d)/dt
    x_dot[3] = Vq - ra * Iq - omega_b * omega * psi_d  # d(psi_q)/dt

    # Rotor flux dynamics
    # Efd from exciter is on field voltage base (≈1 pu at rated conditions)
    # psi_f is the field flux linkage on stator base
    # Convert Efd to stator base: Efd_stator = Efd * Kfd_scale
    # At equilibrium: Efd (exciter pu) = psi_f / Kfd_scale
    x_dot[4] = (Efd * Kfd_scale - psi_f) / Td10  # d(psi_f)/dt - field winding
    x_dot[5] = -psi_kd / Td20  # d(psi_kd)/dt - d-axis damper
    x_dot[6] = -psi_kq / Tq20  # d(psi_kq)/dt - q-axis damper

    return x_dot


def build_genrou_core(gen_data, S_system=100.0):
    """Build GENROU generator as DynamicsCore

    Args:
        gen_data: dict with generator parameters
        S_system: system base power (MVA)

    Returns:
        core: DynamicsCore object with dynamics method
        metadata: dict with additional info
    """
    core = DynamicsCore(label=f'GENROU_{gen_data["idx"]}', dynamics_fn=genrou_dynamics)
    
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
        'D': D_val,
        'Td10': Td10,
        'Td20': Td20,
        'Tq10': Tq10,
        'Tq20': Tq20
    }

    # Set metadata on core for dynamics computation
    core.set_metadata(metadata)
    
    # Set component interface attributes
    core.n_states = 7
    core.output_fn = None  # GENROU outputs are currents, computed in network solution
    core.component_type = "generator"
    core.model_name = "GENROU"

    return core, metadata
