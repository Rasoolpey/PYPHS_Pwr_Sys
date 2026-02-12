"""Utility to register new component models"""


def create_generator_template(model_name):
    """Create template for new generator model"""
    template = f'''"""
{model_name} Generator Model
"""
import sys
sys.path.insert(0, '/home/claude')
from utils.pyphs_core import Core
import numpy as np


def build_{model_name.lower()}_core(gen_data, S_system=100.0):
    """Build {model_name} generator as PyPHS Core
    
    Args:
        gen_data: dict with generator parameters
        S_system: system base power (MVA)
    
    Returns:
        core: PyPHS Core object
        metadata: dict with additional info
    """
    core = Core(label=f'{model_name}_{{gen_data["idx"]}}')
    
    # Extract parameters
    # TODO: Add parameter extraction
    
    # Define symbols
    # TODO: Add state variables
    
    # Build Hamiltonian
    # TODO: Add energy function
    
    # Add dissipations
    # TODO: Add dissipative elements
    
    # Add ports
    # TODO: Add input/output ports
    
    # Parameter substitutions
    core.subs.update({{
        # TODO: Add parameter values
    }})
    
    # Metadata
    metadata = {{
        'idx': gen_data['idx'],
        'bus': gen_data['bus'],
        # TODO: Add metadata
    }}
    
    return core, metadata
'''
    return template


def register_new_model(factory, category, model_type, module_name):
    """Register new model in factory
    
    Args:
        factory: ComponentFactory instance
        category: 'generators', 'exciters', 'governors'
        model_type: model name in JSON (e.g., 'GENSAL')
        module_name: Python module name (e.g., 'gensal')
    
    Example:
        factory = ComponentFactory()
        register_new_model(factory, 'generators', 'GENSAL', 'gensal')
    """
    factory.register_model(category, model_type, module_name)
    print(f"Registered {model_type} in {category} (module: {module_name})")
