"""Component Factory - Dynamically loads and builds components"""
import importlib
import os


class ComponentFactory:
    """Factory for creating power system components from JSON data"""
    
    def __init__(self, components_dir='components'):
        self.components_dir = components_dir
        self.model_registry = {
            'generators': {
                'GENROU': 'genrou'
            },
            'exciters': {
                'EXDC2': 'exdc2'
            },
            'governors': {
                'TGOV1': 'tgov1'
            }
        }
    
    def build_generator(self, model_type, gen_data, S_system=100.0):
        """Build generator component
        
        Args:
            model_type: str, e.g., 'GENROU'
            gen_data: dict with parameters
            S_system: system base power
        
        Returns:
            core, metadata
        """
        module_name = self.model_registry['generators'][model_type]
        module = importlib.import_module(f'components.generators.{module_name}')
        build_func = getattr(module, f'build_{module_name}_core')
        return build_func(gen_data, S_system)
    
    def build_exciter(self, model_type, exc_data):
        """Build exciter component
        
        Args:
            model_type: str, e.g., 'EXDC2'
            exc_data: dict with parameters
        
        Returns:
            core, metadata
        """
        module_name = self.model_registry['exciters'][model_type]
        module = importlib.import_module(f'components.exciters.{module_name}')
        build_func = getattr(module, f'build_{module_name}_core')
        return build_func(exc_data)
    
    def build_governor(self, model_type, gov_data, S_machine=900.0, S_system=100.0):
        """Build governor component
        
        Args:
            model_type: str, e.g., 'TGOV1'
            gov_data: dict with parameters
            S_machine: machine base power
            S_system: system base power
        
        Returns:
            core, metadata
        """
        module_name = self.model_registry['governors'][model_type]
        module = importlib.import_module(f'components.governors.{module_name}')
        build_func = getattr(module, f'build_{module_name}_core')
        return build_func(gov_data, S_machine, S_system)
    
    def build_network(self, system_data):
        """Build network from system data
        
        Args:
            system_data: dict with full system configuration
        
        Returns:
            core, metadata
        """
        from components.network.network_builder import build_network_core
        return build_network_core(system_data)
    
    def register_model(self, category, model_type, module_name):
        """Register new model type
        
        Args:
            category: str, 'generators', 'exciters', 'governors'
            model_type: str, model name in JSON
            module_name: str, Python module name
        """
        if category not in self.model_registry:
            self.model_registry[category] = {}
        self.model_registry[category][model_type] = module_name
