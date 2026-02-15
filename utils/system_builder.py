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
        self.grids = []
        self.network = None

        # Renewable component storage (WT3 sub-components + GFM)
        self.ren_generators = []   # REGCA1 (converter interface)
        self.ren_exciters = []     # REECA1 (electrical control)
        self.ren_plants = []       # REPCA1 (plant controller)
        self.ren_drivetrains = []  # WTDTA1 (drive train)
        self.ren_aero = []         # WTARA1 (aerodynamics)
        self.ren_pitch = []        # WTPTA1 (pitch control)
        self.ren_torque = []       # WTTQA1 (torque control)
        self.ren_voc = []          # VOC_INVERTER (Virtual Oscillator Control grid-forming)

        # Metadata storage
        self.gen_metadata = []
        self.exc_metadata = []
        self.gov_metadata = []
        self.grid_metadata = []
        self.net_metadata = None

        # Renewable metadata storage
        self.ren_gen_metadata = []
        self.ren_exc_metadata = []
        self.ren_plant_metadata = []
        self.ren_dt_metadata = []
        self.ren_aero_metadata = []
        self.ren_pitch_metadata = []
        self.ren_torque_metadata = []
        self.ren_voc_metadata = []

        # Build mappings
        self._build_mappings()

    def _build_mappings(self):
        """Build internal mappings from JSON data"""
        # Generator index to data mapping (dynamically discover all generator types)
        self.gen_idx_to_data = {}
        for model_name in self.factory.category_models.get('generator', []):
            for gen in self.system_data.get(model_name, []):
                self.gen_idx_to_data[gen['idx']] = gen

        # Bus index to data mapping
        self.bus_idx_to_data = {}
        for bus in self.system_data.get('Bus', []):
            self.bus_idx_to_data[bus['idx']] = bus

        # Control to generator mappings (syn field references generator idx)
        # Dynamically discover all exciter types from registry
        self.exc_syn_to_gen = {}
        for model_name in self.factory.category_models.get('exciter', []):
            for exc in self.system_data.get(model_name, []):
                self.exc_syn_to_gen[exc['syn']] = exc

        # Dynamically discover all governor types from registry
        self.gov_syn_to_gen = {}
        for model_name in self.factory.category_models.get('governor', []):
            for gov in self.system_data.get(model_name, []):
                self.gov_syn_to_gen[gov['syn']] = gov

    def build_all_components(self):
        """Build all system components from JSON data - dynamically discovers all component types"""
        print("Building power system components...")

        # Dynamically build generators - scan JSON for any registered generator model
        total_generators = 0
        for model_name in self.factory.category_models.get('generator', []):
            gen_data_list = self.system_data.get(model_name, [])
            if gen_data_list:
                print(f"  Building {len(gen_data_list)} {model_name} generators...")
                for gen_data in gen_data_list:
                    core, meta = self.factory.build_generator(model_name, gen_data, self.S_system)
                    self.generators.append(core)
                    self.gen_metadata.append(meta)
                    total_generators += 1
        
        if total_generators == 0:
            print("  Warning: No generators found in JSON")

        # Dynamically build exciters - scan JSON for any registered exciter model
        total_exciters = 0
        for model_name in self.factory.category_models.get('exciter', []):
            exc_data_list = self.system_data.get(model_name, [])
            if exc_data_list:
                print(f"  Building {len(exc_data_list)} {model_name} exciters...")
                for exc_data in exc_data_list:
                    core, meta = self.factory.build_exciter(model_name, exc_data)
                    self.exciters.append(core)
                    self.exc_metadata.append(meta)
                    total_exciters += 1
        
        if total_exciters == 0:
            print("  Warning: No exciters found in JSON")

        # Dynamically build governors - scan JSON for any registered governor model
        total_governors = 0
        for model_name in self.factory.category_models.get('governor', []):
            gov_data_list = self.system_data.get(model_name, [])
            if gov_data_list:
                print(f"  Building {len(gov_data_list)} {model_name} governors...")
                for gov_data in gov_data_list:
                    # Get machine rating from corresponding generator
                    syn_idx = gov_data['syn']
                    if syn_idx in self.gen_idx_to_data:
                        S_machine = self.gen_idx_to_data[syn_idx]['Sn']
                    else:
                        S_machine = 900.0  # Default
                    
                    core, meta = self.factory.build_governor(model_name, gov_data, S_machine, self.S_system)
                    self.governors.append(core)
                    self.gov_metadata.append(meta)
                    total_governors += 1
        
        if total_governors == 0:
            print("  Warning: No governors found in JSON")

        # Dynamically build grid/slack buses (only for TRUE grids without generators)
        # Get generator bus mapping to check which slacks are true grids
        gen_buses = set()
        for gen_meta in self.gen_metadata:
            gen_buses.add(gen_meta['bus'])
        
        total_grids = 0
        for model_name in self.factory.category_models.get('grid', []):
            grid_data_list = self.system_data.get(model_name, [])
            if grid_data_list:
                true_grids = []
                for grid in grid_data_list:
                    bus_idx = grid['bus']
                    if bus_idx not in gen_buses:
                        # This is a true grid (no generator at this bus)
                        true_grids.append(grid)
                
                if true_grids:
                    print(f"  Building {len(true_grids)} {model_name} grid sources...")
                    for grid_data in true_grids:
                        core, meta = self.factory.build_grid(model_name, grid_data)
                        self.grids.append(core)
                        self.grid_metadata.append(meta)
                        total_grids += 1
        
        if total_grids == 0:
            print("  No true grid buses (all slacks are reference generators)")

        # Dynamically build renewable components
        self._build_renewables()

        # Build network
        print("  Building network from Line data...")
        self.network, self.net_metadata = self.factory.build_network(self.system_data)

        print("Component build complete!")
        self._print_build_summary()
        return self

    def _build_renewables(self):
        """Build all renewable energy components from JSON data"""
        ren_model_map = {
            'REGCA1': ('ren_generators', 'ren_gen_metadata'),
            'REECA1': ('ren_exciters', 'ren_exc_metadata'),
            'REPCA1': ('ren_plants', 'ren_plant_metadata'),
            'WTDTA1': ('ren_drivetrains', 'ren_dt_metadata'),
            'WTARA1': ('ren_aero', 'ren_aero_metadata'),
            'WTPTA1': ('ren_pitch', 'ren_pitch_metadata'),
            'WTTQA1': ('ren_torque', 'ren_torque_metadata'),
            'VOC_INVERTER': ('ren_voc', 'ren_voc_metadata'),
        }

        total_ren = 0
        for model_name, (core_list_name, meta_list_name) in ren_model_map.items():
            data_list = self.system_data.get(model_name, [])
            if data_list:
                print(f"  Building {len(data_list)} {model_name} renewable components...")
                for data in data_list:
                    core, meta = self.factory.build_renewable(model_name, data, self.S_system)
                    getattr(self, core_list_name).append(core)
                    getattr(self, meta_list_name).append(meta)
                    total_ren += 1

        if total_ren > 0:
            print(f"  Total renewable components: {total_ren}")

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
        counts = {
            'generators': len(self.generators),
            'exciters': len(self.exciters),
            'governors': len(self.governors),
            'grids': len(self.grids),
            'buses': self.net_metadata.get('n_bus', 0) if self.net_metadata else 0,
            'network_lines': len(self.net_metadata.get('line_configs', [])) if self.net_metadata else 0,
            'ren_generators': len(self.ren_generators),
            'ren_exciters': len(self.ren_exciters),
            'ren_plants': len(self.ren_plants),
            'ren_drivetrains': len(self.ren_drivetrains),
            'ren_aero': len(self.ren_aero),
            'ren_pitch': len(self.ren_pitch),
            'ren_torque': len(self.ren_torque),
            'ren_voc': len(self.ren_voc),
        }
        counts['total_renewables'] = sum([
            counts['ren_generators'], counts['ren_exciters'], counts['ren_plants'],
            counts['ren_drivetrains'], counts['ren_aero'], counts['ren_pitch'],
            counts['ren_torque'], counts['ren_voc']
        ])
        return counts

    def get_total_states(self):
        """Calculate total number of states"""
        n_states = 0
        n_states += sum(len(g.x) for g in self.generators)
        n_states += sum(len(e.x) for e in self.exciters)
        n_states += sum(len(gov.x) for gov in self.governors)
        if self.network:
            n_states += len(self.network.x)
        # Renewable component states
        for ren_list in [self.ren_generators, self.ren_exciters, self.ren_plants,
                         self.ren_drivetrains, self.ren_pitch, self.ren_torque]:
            n_states += sum(c.n_states for c in ren_list)
        # WTARA1 has 0 states (algebraic), but count anyway for safety
        n_states += sum(c.n_states for c in self.ren_aero)
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

    def get_renewable_mapping(self):
        """Return mapping of WT3 sub-components to each other.

        The WT3 hierarchy is:
          REPCA1 -> REECA1 -> REGCA1 -> Network bus
          WTDTA1 (drive train) + WTARA1 (aero) + WTPTA1 (pitch) + WTTQA1 (torque)

        Links are via 'ree' (REECA1 idx) and 'gen' (REGCA1 idx) fields in JSON.
        """
        mapping = []
        for i, reg_meta in enumerate(self.ren_gen_metadata):
            entry = {
                'regca1_idx': i,
                'bus': reg_meta.get('bus', None),
                'reeca1_idx': None,
                'repca1_idx': None,
                'wtdta1_idx': None,
                'wtara1_idx': None,
                'wtpta1_idx': None,
                'wttqa1_idx': None,
            }
            reg_json_idx = reg_meta.get('idx', None)
            ree_json_idx = None  # Initialize before loop
            aero_json_idx = None  # Initialize before loop

            # Find REECA1 linked to this REGCA1
            # REECA1 uses "reg" field to link to REGCA1
            for j, ree_meta in enumerate(self.ren_exc_metadata):
                if ree_meta.get('reg', None) == reg_json_idx:
                    entry['reeca1_idx'] = j
                    ree_json_idx = ree_meta.get('idx', None)

                    # Find REPCA1 linked to this REECA1
                    for k, rep_meta in enumerate(self.ren_plant_metadata):
                        if rep_meta.get('ree', None) == ree_json_idx:
                            entry['repca1_idx'] = k
                            rep_json_idx = rep_meta.get('idx', None)
                            
                            # Find WTTQA1 linked to this REPCA1
                            for m, torque_meta in enumerate(self.ren_torque_metadata):
                                if torque_meta.get('rep', None) == rep_json_idx:
                                    entry['wttqa1_idx'] = m
                                    break
                            break
                    break

            # Find WTDTA1 linked to REECA1
            if ree_json_idx is not None:
                for j, dt_meta in enumerate(self.ren_dt_metadata):
                    if dt_meta.get('ree', None) == ree_json_idx:
                        entry['wtdta1_idx'] = j
                        break

            # Find WTARA1 linked to REGCA1 (uses "rego" field)
            for j, aero_meta in enumerate(self.ren_aero_metadata):
                if aero_meta.get('rego', None) == reg_json_idx:
                    entry['wtara1_idx'] = j
                    aero_json_idx = aero_meta.get('idx', None)
                    
                    # Find WTPTA1 linked to WTARA1 (uses "rea" field)
                    if aero_json_idx is not None:
                        for k, pitch_meta in enumerate(self.ren_pitch_metadata):
                            if pitch_meta.get('rea', None) == aero_json_idx:
                                entry['wtpta1_idx'] = k
                                break
                    break

            mapping.append(entry)
        return mapping

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
        print(f"Grids:      {counts['grids']}")
        print(f"Lines:      {counts['network_lines']}")
        if counts['total_renewables'] > 0:
            print(f"Renewables: {counts['total_renewables']} components")
            print(f"  REGCA1: {counts['ren_generators']}, REECA1: {counts['ren_exciters']}, "
                  f"REPCA1: {counts['ren_plants']}")
            print(f"  WTDTA1: {counts['ren_drivetrains']}, WTARA1: {counts['ren_aero']}, "
                  f"WTPTA1: {counts['ren_pitch']}, WTTQA1: {counts['ren_torque']}")
        print(f"Total states: {n_states}")
        print("-"*50)

        # Generator details
        print("\nGenerator Mapping:")
        gen_bus_map = self.get_generator_bus_mapping()
        for gen_idx, bus_idx in gen_bus_map.items():
            exc_idx = control_map['exciters'].get(gen_idx, 'N/A')
            gov_idx = control_map['governors'].get(gen_idx, 'N/A')
            print(f"  Gen {gen_idx} -> Bus {bus_idx}, Exciter: {exc_idx}, Governor: {gov_idx}")

        # Renewable details
        if counts['total_renewables'] > 0:
            print("\nRenewable Mapping:")
            ren_map = self.get_renewable_mapping()
            for entry in ren_map:
                print(f"  REGCA1 {entry['regca1_idx']} -> Bus {entry['bus']}, "
                      f"REECA1: {entry['reeca1_idx']}, REPCA1: {entry['repca1_idx']}, "
                      f"WTDTA1: {entry['wtdta1_idx']}")

        print("="*50 + "\n")

        return self
