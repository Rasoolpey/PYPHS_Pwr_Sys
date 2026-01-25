"""System Builder - Constructs full power system from JSON configuration"""
import json
import numpy as np
from utils.component_factory import ComponentFactory


class PowerSystemBuilder:
    """Builds complete power system from JSON configuration"""
    
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
        
        # Build exciters
        exdc2_data = self.system_data.get('EXDC2', [])
        print(f"  Building {len(exdc2_data)} EXDC2 exciters...")
        for exc_data in exdc2_data:
            core, meta = self.factory.build_exciter('EXDC2', exc_data)
            self.exciters.append(core)
            self.exc_metadata.append(meta)
        
        # Build governors
        tgov1_data = self.system_data.get('TGOV1', [])
        print(f"  Building {len(tgov1_data)} TGOV1 governors...")
        for gov_data in tgov1_data:
            S_machine = genrou_data[gov_data['syn']-1]['Sn']
            core, meta = self.factory.build_governor('TGOV1', gov_data, S_machine, self.S_system)
            self.governors.append(core)
            self.gov_metadata.append(meta)
        
        # Build network
        print("  Building network...")
        self.network, self.net_metadata = self.factory.build_network(self.system_data)
        
        print("Component build complete!")
        return self
    
    def get_component_counts(self):
        """Return component counts"""
        return {
            'generators': len(self.generators),
            'exciters': len(self.exciters),
            'governors': len(self.governors),
            'network_lines': len(self.net_metadata['line_configs']) if self.net_metadata else 0
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
    
    def get_control_mapping(self):
        """Return mapping of controls to generators"""
        exciter_map = {}
        governor_map = {}
        
        for i, meta in enumerate(self.exc_metadata):
            syn_idx = meta['syn'] - 1
            exciter_map[syn_idx] = i
        
        for i, meta in enumerate(self.gov_metadata):
            syn_idx = meta['syn'] - 1
            governor_map[syn_idx] = i
        
        return {'exciters': exciter_map, 'governors': governor_map}
    
    def summary(self):
        """Print system summary"""
        counts = self.get_component_counts()
        n_states = self.get_total_states()
        
        print("\n=== Power System Summary ===")
        print(f"Generators: {counts['generators']}")
        print(f"Exciters: {counts['exciters']}")
        print(f"Governors: {counts['governors']}")
        print(f"Network lines: {counts['network_lines']}")
        print(f"Total states: {n_states}")
        print("===========================\n")
        
        return self
