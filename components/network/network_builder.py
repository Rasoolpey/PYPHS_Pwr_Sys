"""
Network Builder - Constructs network from JSON data dynamically
Builds PHS network model from actual Line data without hardcoded topology
"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from utils.pyphs_core import Core
import numpy as np


def build_network_core(system_data):
    """Build network as PyPHS Core from system data

    Dynamically constructs network topology from Line data in JSON.
    Handles transmission lines, transformers, and loads.

    Args:
        system_data: dict with full system configuration

    Returns:
        core: PyPHS Core object
        metadata: dict with network info
    """
    core = Core(label='Network')

    # Extract data from JSON
    buses = system_data.get('Bus', [])
    lines = system_data.get('Line', [])
    generators = system_data.get('GENROU', [])
    loads = system_data.get('PQ', [])

    # Build bus mapping
    bus_idx_to_internal = {}
    for i, bus in enumerate(sorted(buses, key=lambda b: b['idx'])):
        bus_idx_to_internal[bus['idx']] = i

    n_bus = len(buses)
    n_gen = len(generators)

    # Get generator buses
    gen_buses = [g['bus'] for g in generators]

    # Build load dictionary
    load_dict = {}
    for load in loads:
        bus_idx = load['bus']
        if bus_idx not in load_dict:
            load_dict[bus_idx] = {'p0': 0, 'q0': 0}
        load_dict[bus_idx]['p0'] += load['p0']
        load_dict[bus_idx]['q0'] += load['q0']

    # Process lines - separate transmission lines and transformers
    transmission_lines = []
    transformers = []

    for line in lines:
        line_data = {
            'idx': line['idx'],
            'bus1': line['bus1'],
            'bus2': line['bus2'],
            'r': line['r'],
            'x': line['x'],
            'b': line.get('b', 0.0),
            'Vn1': line.get('Vn1', 1.0),
            'Vn2': line.get('Vn2', 1.0),
            'tap': line.get('tap', 1.0)
        }

        if line_data['Vn1'] != line_data['Vn2']:
            transformers.append(line_data)
        else:
            transmission_lines.append(line_data)

    # Group parallel lines by bus pair
    line_groups = {}
    for line in transmission_lines:
        key = tuple(sorted([line['bus1'], line['bus2']]))
        if key not in line_groups:
            line_groups[key] = []
        line_groups[key].append(line)

    # Create symbols and build Hamiltonian for each unique connection
    flux_symbols = []
    line_configs = []

    # Process transmission line groups (parallel lines become single equivalent)
    for (bus1, bus2), group_lines in line_groups.items():
        # Calculate equivalent impedance for parallel lines
        y_total = 0
        b_total = 0
        for ln in group_lines:
            z = complex(ln['r'], ln['x'])
            if abs(z) > 1e-10:
                y_total += 1.0 / z
            b_total += ln['b']

        if abs(y_total) > 1e-10:
            z_eq = 1.0 / y_total
            r_eq = z_eq.real
            x_eq = z_eq.imag
        else:
            r_eq = group_lines[0]['r']
            x_eq = group_lines[0]['x']

        # Create flux linkage symbol for this connection
        line_name = f'line_{bus1}_{bus2}'
        psi = core.symbols(f'psi_{line_name}')
        flux_symbols.append(psi)

        line_configs.append({
            'name': line_name,
            'bus1': bus1,
            'bus2': bus2,
            'bus1_internal': bus_idx_to_internal.get(bus1),
            'bus2_internal': bus_idx_to_internal.get(bus2),
            'L': x_eq,  # Inductance (reactance in pu)
            'R': r_eq,
            'B': b_total,
            'n_parallel': len(group_lines)
        })

    # Process transformers
    for xfmr in transformers:
        line_name = f'xfmr_{xfmr["bus1"]}_{xfmr["bus2"]}'
        psi = core.symbols(f'psi_{line_name}')
        flux_symbols.append(psi)

        line_configs.append({
            'name': line_name,
            'bus1': xfmr['bus1'],
            'bus2': xfmr['bus2'],
            'bus1_internal': bus_idx_to_internal.get(xfmr['bus1']),
            'bus2_internal': bus_idx_to_internal.get(xfmr['bus2']),
            'L': xfmr['x'],
            'R': xfmr['r'],
            'B': 0,
            'ratio': xfmr['Vn2'] / xfmr['Vn1'],
            'is_transformer': True
        })

    # Build Hamiltonian: magnetic energy in transmission lines
    H_net = 0
    L_symbols = {}

    for i, config in enumerate(line_configs):
        L_sym = core.symbols(f'L_{config["name"]}')
        L_symbols[config['name']] = L_sym
        H_net += flux_symbols[i]**2 / (2 * L_sym)

    if flux_symbols:
        core.add_storages(flux_symbols, H_net)

    # Dissipations: line resistances
    w_dissipations = []
    z_dissipations = []
    R_symbols = {}

    for config in line_configs:
        w_i = core.symbols(f'w_{config["name"]}')
        R_sym = core.symbols(f'R_{config["name"]}')
        R_symbols[config['name']] = R_sym
        z_i = R_sym * w_i

        w_dissipations.append(w_i)
        z_dissipations.append(z_i)

    # Load dissipations at each bus with load
    load_P_array = np.zeros(n_bus)
    load_Q_array = np.zeros(n_bus)

    for bus_idx, load_data in load_dict.items():
        if bus_idx in bus_idx_to_internal:
            internal_idx = bus_idx_to_internal[bus_idx]
            load_P_array[internal_idx] = load_data['p0']
            load_Q_array[internal_idx] = load_data['q0']

    # Add load dissipation at generator buses
    for i, bus_idx in enumerate(gen_buses):
        w_load_i = core.symbols(f'w_load_gen_{i}')

        # Get load at this bus
        if bus_idx in load_dict:
            P_load = load_dict[bus_idx]['p0']
        else:
            P_load = 0.1  # Small default load

        R_load_i = 1.0 / max(abs(P_load), 0.1)
        z_load_i = R_load_i * w_load_i

        w_dissipations.append(w_load_i)
        z_dissipations.append(z_load_i)

    if w_dissipations:
        core.add_dissipations(w_dissipations, z_dissipations)

    # Ports: generator connections
    V_inputs = []
    I_outputs = []
    for i in range(n_gen):
        V_i = core.symbols(f'V_gen_{i}')
        I_i = core.symbols(f'I_gen_{i}')
        V_inputs.append(V_i)
        I_outputs.append(I_i)

    if V_inputs:
        core.add_ports(V_inputs, I_outputs)

    # Parameter substitutions
    for config in line_configs:
        L_sym = L_symbols[config['name']]
        R_sym = R_symbols[config['name']]
        core.subs[L_sym] = config['L']
        core.subs[R_sym] = config['R']

    # Metadata
    metadata = {
        'n_bus': n_bus,
        'n_gen': n_gen,
        'gen_buses': gen_buses,
        'bus_idx_to_internal': bus_idx_to_internal,
        'line_configs': line_configs,
        'n_transmission_lines': len(transmission_lines),
        'n_transformers': len(transformers),
        'n_unique_connections': len(line_groups) + len(transformers),
        'load_P': load_P_array.tolist(),
        'load_Q': load_Q_array.tolist(),
        'load_dict': load_dict
    }

    return core, metadata


def build_ybus_from_lines(system_data):
    """
    Build Ybus matrix directly from Line data

    Args:
        system_data: dict with system configuration

    Returns:
        Ybus: complex admittance matrix
        bus_mapping: dict mapping bus idx to matrix index
    """
    buses = system_data.get('Bus', [])
    lines = system_data.get('Line', [])

    # Build bus mapping
    bus_idx_to_internal = {}
    for i, bus in enumerate(sorted(buses, key=lambda b: b['idx'])):
        bus_idx_to_internal[bus['idx']] = i

    n_bus = len(buses)
    Ybus = np.zeros((n_bus, n_bus), dtype=complex)

    for line in lines:
        bus1_idx = line['bus1']
        bus2_idx = line['bus2']

        if bus1_idx not in bus_idx_to_internal or bus2_idx not in bus_idx_to_internal:
            continue

        i = bus_idx_to_internal[bus1_idx]
        j = bus_idx_to_internal[bus2_idx]

        r = line['r']
        x = line['x']
        b = line.get('b', 0.0)
        Vn1 = line.get('Vn1', 1.0)
        Vn2 = line.get('Vn2', 1.0)
        tap = line.get('tap', 1.0)

        # Series admittance
        z_series = complex(r, x)
        if abs(z_series) > 1e-10:
            y_series = 1.0 / z_series
        else:
            y_series = complex(0, -1e6)

        # Shunt admittance
        y_shunt = complex(0, b / 2)

        if Vn1 != Vn2:
            # Transformer
            n = (Vn2 / Vn1) * tap
            Ybus[i, i] += y_series / (n * n)
            Ybus[j, j] += y_series
            Ybus[i, j] -= y_series / n
            Ybus[j, i] -= y_series / n
        else:
            # Transmission line
            Ybus[i, i] += y_series + y_shunt
            Ybus[j, j] += y_series + y_shunt
            Ybus[i, j] -= y_series
            Ybus[j, i] -= y_series

    return Ybus, bus_idx_to_internal


def get_network_topology(system_data):
    """
    Extract network topology information from JSON data

    Args:
        system_data: dict with system configuration

    Returns:
        topology: dict with connectivity information
    """
    buses = system_data.get('Bus', [])
    lines = system_data.get('Line', [])
    generators = system_data.get('GENROU', [])
    loads = system_data.get('PQ', [])

    # Build adjacency information
    adjacency = {}
    for bus in buses:
        adjacency[bus['idx']] = {
            'neighbors': [],
            'lines': [],
            'generators': [],
            'loads': [],
            'voltage': bus['Vn'],
            'area': bus.get('area', 1)
        }

    # Add line connections
    for line in lines:
        b1, b2 = line['bus1'], line['bus2']
        if b1 in adjacency:
            adjacency[b1]['neighbors'].append(b2)
            adjacency[b1]['lines'].append(line['idx'])
        if b2 in adjacency:
            adjacency[b2]['neighbors'].append(b1)
            adjacency[b2]['lines'].append(line['idx'])

    # Add generators
    for gen in generators:
        bus_idx = gen['bus']
        if bus_idx in adjacency:
            adjacency[bus_idx]['generators'].append(gen['idx'])

    # Add loads
    for load in loads:
        bus_idx = load['bus']
        if bus_idx in adjacency:
            adjacency[bus_idx]['loads'].append(load['idx'])

    return {
        'adjacency': adjacency,
        'n_buses': len(buses),
        'n_lines': len(lines),
        'n_generators': len(generators),
        'n_loads': len(loads)
    }
