"""System Builder - Constructs full power system from JSON configuration dynamically"""
import json
import numpy as np
from utils.component_factory import ComponentFactory


class PowerSystemBuilder:
    """Builds complete power system from JSON configuration dynamically"""

    def __init__(self, json_file):
        """Initialize from JSON file

        Args:
            json_file: path to system configuration JSON
        """
        with open(json_file, 'r') as f:
            self.system_data = json.load(f)

        self.factory = ComponentFactory()
        self.S_system = 100.0

        # Component storage
        self.generators = []
        self.exciters = []
        self.governors = []
        self.network = None

        # Metadata storage
        self.gen_metadata = []
        self.exc_metadata = []
        self.gov_metadata = []
        self.net_metadata = None

        # Build mappings
        self._build_mappings()

    def _build_mappings(self):
        """Build internal mappings from JSON data"""
        # Generator index to data mapping
        self.gen_idx_to_data = {}
        for gen in self.system_data.get('GENROU', []):
            self.gen_idx_to_data[gen['idx']] = gen

        # Bus index to data mapping
        self.bus_idx_to_data = {}
        for bus in self.system_data.get('Bus', []):
            self.bus_idx_to_data[bus['idx']] = bus

        # Control to generator mappings (syn field references generator idx)
        self.exc_syn_to_gen = {}
        for exc in self.system_data.get('EXDC2', []):
            self.exc_syn_to_gen[exc['syn']] = exc

        self.gov_syn_to_gen = {}
        for gov in self.system_data.get('TGOV1', []):
            self.gov_syn_to_gen[gov['syn']] = gov

    def build_all_components(self):
        """Build all system components from JSON data"""
        print("Building power system components...")

        # Build generators
        genrou_data = self.system_data.get('GENROU', [])
        print(f"  Building {len(genrou_data)} GENROU generators...")
        for gen_data in genrou_data:
            core, meta = self.factory.build_generator('GENROU', gen_data, self.S_system)
            self.generators.append(core)
            self.gen_metadata.append(meta)

        # Build exciters - match to generators via 'syn' field
        exdc2_data = self.system_data.get('EXDC2', [])
        print(f"  Building {len(exdc2_data)} EXDC2 exciters...")
        for exc_data in exdc2_data:
            core, meta = self.factory.build_exciter('EXDC2', exc_data)
            self.exciters.append(core)
            self.exc_metadata.append(meta)

        # Build governors - match to generators via 'syn' field
        tgov1_data = self.system_data.get('TGOV1', [])
        print(f"  Building {len(tgov1_data)} TGOV1 governors...")
        for gov_data in tgov1_data:
            # Get machine rating from corresponding generator
            syn_idx = gov_data['syn']
            if syn_idx in self.gen_idx_to_data:
                S_machine = self.gen_idx_to_data[syn_idx]['Sn']
            else:
                # Fallback: search by position
                gen_list = self.system_data.get('GENROU', [])
                if syn_idx - 1 < len(gen_list):
                    S_machine = gen_list[syn_idx - 1]['Sn']
                else:
                    S_machine = 900.0  # Default

            core, meta = self.factory.build_governor('TGOV1', gov_data, S_machine, self.S_system)
            self.governors.append(core)
            self.gov_metadata.append(meta)

        # Build network
        print("  Building network from Line data...")
        self.network, self.net_metadata = self.factory.build_network(self.system_data)

        print("Component build complete!")
        self._print_build_summary()
        return self

    def _print_build_summary(self):
        """Print summary of built components"""
        print(f"\n  Network topology:")
        if self.net_metadata:
            print(f"    - {self.net_metadata.get('n_bus', 0)} buses")
            print(f"    - {self.net_metadata.get('n_transmission_lines', 0)} transmission lines")
            print(f"    - {self.net_metadata.get('n_transformers', 0)} transformers")
            print(f"    - {self.net_metadata.get('n_unique_connections', 0)} unique connections")

    def get_component_counts(self):
        """Return component counts"""
        return {
            'generators': len(self.generators),
            'exciters': len(self.exciters),
            'governors': len(self.governors),
            'buses': self.net_metadata.get('n_bus', 0) if self.net_metadata else 0,
            'network_lines': len(self.net_metadata.get('line_configs', [])) if self.net_metadata else 0
        }

    def get_total_states(self):
        """Calculate total number of states"""
        n_states = 0
        n_states += sum(len(g.x) for g in self.generators)
        n_states += sum(len(e.x) for e in self.exciters)
        n_states += sum(len(gov.x) for gov in self.governors)
        if self.network:
            n_states += len(self.network.x)
        return n_states

    def get_generator_bus_mapping(self):
        """Return mapping of generator index to bus"""
        return {i: meta['bus'] for i, meta in enumerate(self.gen_metadata)}

    def get_bus_generator_mapping(self):
        """Return mapping of bus to list of generator indices"""
        bus_to_gens = {}
        for i, meta in enumerate(self.gen_metadata):
            bus_idx = meta['bus']
            if bus_idx not in bus_to_gens:
                bus_to_gens[bus_idx] = []
            bus_to_gens[bus_idx].append(i)
        return bus_to_gens

    def get_control_mapping(self):
        """Return mapping of controls to generators

        The 'syn' field in exciter/governor data references the generator idx.
        We need to map this to our internal generator array index.
        """
        # Build gen_idx to array position mapping
        gen_idx_to_pos = {}
        for i, meta in enumerate(self.gen_metadata):
            gen_idx_to_pos[meta['idx']] = i

        exciter_map = {}
        for i, meta in enumerate(self.exc_metadata):
            syn_idx = meta['syn']
            if syn_idx in gen_idx_to_pos:
                gen_pos = gen_idx_to_pos[syn_idx]
                exciter_map[gen_pos] = i

        governor_map = {}
        for i, meta in enumerate(self.gov_metadata):
            syn_idx = meta['syn']
            if syn_idx in gen_idx_to_pos:
                gen_pos = gen_idx_to_pos[syn_idx]
                governor_map[gen_pos] = i

        return {'exciters': exciter_map, 'governors': governor_map}

    def get_line_data(self):
        """Return processed line data from network metadata"""
        if self.net_metadata:
            return self.net_metadata.get('line_configs', [])
        return []

    def get_load_data(self):
        """Return load data from network metadata"""
        if self.net_metadata:
            return {
                'P': self.net_metadata.get('load_P', []),
                'Q': self.net_metadata.get('load_Q', []),
                'by_bus': self.net_metadata.get('load_dict', {})
            }
        return {'P': [], 'Q': [], 'by_bus': {}}

    def summary(self):
        """Print system summary"""
        counts = self.get_component_counts()
        n_states = self.get_total_states()
        control_map = self.get_control_mapping()

        print("\n" + "="*50)
        print("         POWER SYSTEM SUMMARY")
        print("="*50)
        print(f"Buses:      {counts['buses']}")
        print(f"Generators: {counts['generators']}")
        print(f"Exciters:   {counts['exciters']}")
        print(f"Governors:  {counts['governors']}")
        print(f"Lines:      {counts['network_lines']}")
        print(f"Total states: {n_states}")
        print("-"*50)

        # Generator details
        print("\nGenerator Mapping:")
        gen_bus_map = self.get_generator_bus_mapping()
        for gen_idx, bus_idx in gen_bus_map.items():
            exc_idx = control_map['exciters'].get(gen_idx, 'N/A')
            gov_idx = control_map['governors'].get(gen_idx, 'N/A')
            print(f"  Gen {gen_idx} -> Bus {bus_idx}, Exciter: {exc_idx}, Governor: {gov_idx}")

        print("="*50 + "\n")

        return self
