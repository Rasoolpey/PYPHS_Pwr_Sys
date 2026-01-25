"""Network Builder - Constructs network from JSON data"""
import sys
sys.path.insert(0, '/home/claude')
from utils.pyphs_core import Core
import numpy as np


def build_network_core(system_data):
    """Build network as PyPHS Core from system data
    
    Args:
        system_data: dict with full system configuration
    
    Returns:
        core: PyPHS Core object
        metadata: dict with network info
    """
    core = Core(label='Network')
    
    # Extract network config
    net_config = system_data.get('network_config', {})
    
    # Get generator buses
    generators = system_data.get('GENROU', [])
    gen_buses = [g['bus'] for g in generators]
    n_gen = len(gen_buses)
    
    # Get load data
    loads = system_data.get('PQ', [])
    load_dict = {load['bus']: load for load in loads}
    
    # Network parameters
    X_intra = net_config.get('X_intra', 0.074)
    R_intra = net_config.get('R_intra', 0.0)
    X_tie = net_config.get('X_tie', 0.220)
    R_tie = net_config.get('R_tie', 0.0)
    
    L_intra = X_intra
    L_tie = X_tie
    
    # Flux linkages for transmission lines
    flux_symbols = []
    line_configs = []
    
    # Determine topology from generator buses
    # For 4-generator Kundur: Gen1-Gen2 (intra), Gen3-Gen4 (intra), Gen1-Gen3 (tie)
    if n_gen == 4:
        # Intra-area line Gen1-Gen2
        psi_12 = core.symbols('psi_12')
        flux_symbols.append(psi_12)
        line_configs.append({'name': 'intra_12', 'gen_idx': [0, 1], 'L': L_intra, 'R': R_intra})
        
        # Intra-area line Gen3-Gen4
        psi_34 = core.symbols('psi_34')
        flux_symbols.append(psi_34)
        line_configs.append({'name': 'intra_34', 'gen_idx': [2, 3], 'L': L_intra, 'R': R_intra})
        
        # Tie line Gen1-Gen3
        psi_13 = core.symbols('psi_13')
        flux_symbols.append(psi_13)
        line_configs.append({'name': 'tie_13', 'gen_idx': [0, 2], 'L': L_tie, 'R': R_tie})
    
    # Parameters
    L_intra_sym = core.symbols('L_intra')
    R_intra_sym = core.symbols('R_intra')
    L_tie_sym = core.symbols('L_tie')
    R_tie_sym = core.symbols('R_tie')
    
    # Hamiltonian: magnetic energy in transmission lines
    H_net = 0
    for i, config in enumerate(line_configs):
        if 'tie' in config['name']:
            H_net += flux_symbols[i]**2 / (2 * L_tie_sym)
        else:
            H_net += flux_symbols[i]**2 / (2 * L_intra_sym)
    
    core.add_storages(flux_symbols, H_net)
    
    # Dissipations: line resistances + loads
    w_dissipations = []
    z_dissipations = []
    
    for i, config in enumerate(line_configs):
        w_i = core.symbols(f'w_{config["name"]}')
        if 'tie' in config['name']:
            z_i = R_tie_sym * w_i
        else:
            z_i = R_intra_sym * w_i
        w_dissipations.append(w_i)
        z_dissipations.append(z_i)
    
    # Load dissipations
    load_P = []
    for i in range(n_gen):
        bus_idx = gen_buses[i]
        if bus_idx in load_dict:
            load_P.append(load_dict[bus_idx]['p0'])
        else:
            load_P.append(0.1)
    
    for i in range(n_gen):
        w_load_i = core.symbols(f'w_load_{i}')
        R_load_i = 1.0 / max(load_P[i], 0.1)
        z_load_i = R_load_i * w_load_i
        w_dissipations.append(w_load_i)
        z_dissipations.append(z_load_i)
    
    core.add_dissipations(w_dissipations, z_dissipations)
    
    # Ports: generator connections
    V_inputs = []
    I_outputs = []
    for i in range(n_gen):
        V_i = core.symbols(f'V_gen_{i}')
        I_i = core.symbols(f'I_gen_{i}')
        V_inputs.append(V_i)
        I_outputs.append(I_i)
    
    core.add_ports(V_inputs, I_outputs)
    
    # Parameter substitutions
    core.subs.update({
        L_intra_sym: L_intra,
        R_intra_sym: R_intra,
        L_tie_sym: L_tie,
        R_tie_sym: R_tie
    })
    
    # Metadata
    metadata = {
        'n_gen': n_gen,
        'gen_buses': gen_buses,
        'X_intra': X_intra,
        'X_tie': X_tie,
        'load_P': load_P,
        'line_configs': line_configs
    }
    
    return core, metadata
