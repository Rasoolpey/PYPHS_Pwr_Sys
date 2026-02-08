"""Component Factory - Dynamically loads and builds components"""
import importlib
import os


class ComponentFactory:
    """Factory for creating power system components from JSON data
    
    The registry maps JSON model names to their implementation details.
    Format: {model_name: {'category': category, 'module': module_name}}
    """
    
    def __init__(self, components_dir='components'):
        self.components_dir = components_dir
        
        # Hierarchical registry: model_name -> {category, module}
        # This enables automatic discovery from JSON
        self.model_registry = {
            # Generators
            'GENROU': {'category': 'generator', 'module': 'genrou'},
            
            # Exciters
            'EXDC2': {'category': 'exciter', 'module': 'exdc2'},
            'ESST3A': {'category': 'exciter', 'module': 'esst3a'},
            
            # Governors
            'TGOV1': {'category': 'governor', 'module': 'tgov1'},
            
            # Grid elements
            'Slack': {'category': 'grid', 'module': 'infinite_bus'},
        }
        
        # Reverse lookup: category -> list of model names
        self.category_models = {}
        for model_name, info in self.model_registry.items():
            category = info['category']
            if category not in self.category_models:
                self.category_models[category] = []
            self.category_models[category].append(model_name)
    
    def get_model_info(self, model_name):
        """Get model information from registry
        
        Args:
            model_name: str, e.g., 'GENROU', 'ESST3A'
        
        Returns:
            dict with 'category' and 'module' keys, or None if not found
        """
        return self.model_registry.get(model_name)
    
    def build_generator(self, model_type, gen_data, S_system=100.0):
        """Build generator component
        
        Args:
            model_type: str, e.g., 'GENROU'
            gen_data: dict with parameters
            S_system: system base power
        
        Returns:
            core, metadata
        """
        model_info = self.model_registry.get(model_type)
        if not model_info or model_info['category'] != 'generator':
            raise ValueError(f"Unknown generator model: {model_type}")
        
        module_name = model_info['module']
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
        model_info = self.model_registry.get(model_type)
        if not model_info or model_info['category'] != 'exciter':
            raise ValueError(f"Unknown exciter model: {model_type}")
        
        module_name = model_info['module']
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
        model_info = self.model_registry.get(model_type)
        if not model_info or model_info['category'] != 'governor':
            raise ValueError(f"Unknown governor model: {model_type}")
        
        module_name = model_info['module']
        module = importlib.import_module(f'components.governors.{module_name}')
        build_func = getattr(module, f'build_{module_name}_core')
        return build_func(gov_data, S_machine, S_system)
    
    def build_grid(self, model_type, grid_data):
        """Build grid/slack bus component
        
        Args:
            model_type: str, e.g., 'InfiniteBus' or 'Slack'
            grid_data: dict with parameters
        
        Returns:
            core, metadata
        """
        model_info = self.model_registry.get(model_type)
        if not model_info or model_info['category'] != 'grid':
            raise ValueError(f"Unknown grid model: {model_type}")
        
        module_name = model_info['module']
        module = importlib.import_module(f'components.grid.{module_name}')
        build_func = getattr(module, f'build_{module_name}_core')
        return build_func(grid_data)
    
    def build_network(self, system_data):
        """Build network from system data
        
        Args:
            system_data: dict with full system configuration
        
        Returns:
            core, metadata
        """
        from components.network.network_builder import build_network_core
        return build_network_core(system_data)
    
    def register_model(self, model_type, category, module_name):
        """Register new model type
        
        Args:
            model_type: str, model name in JSON (e.g., 'GENROU')
            category: str, 'generator', 'exciter', 'governor', 'grid'
            module_name: str, Python module name (e.g., 'genrou')
        """
        self.model_registry[model_type] = {
            'category': category,
            'module': module_name
        }
        
        # Update reverse lookup
        if category not in self.category_models:
            self.category_models[category] = []
        if model_type not in self.category_models[category]:
            self.category_models[category].append(model_type)
