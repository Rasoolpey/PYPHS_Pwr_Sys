# Component Development Guide

This guide explains how to create new component models for the Port-Hamiltonian power system framework.

## Overview

The framework uses a modular architecture where components are:
- **Self-describing**: Components specify their own state count and output functions
- **Dynamically loaded**: New components are automatically discovered from JSON
- **Pluggable**: Components can be swapped by changing the JSON configuration

## Component Structure

Each component model consists of three main parts:

### 1. Dynamics Function

Computes state derivatives given current states and ports.

```python
def component_dynamics(x, ports, meta):
    """
    Component dynamics function.
    
    Args:
        x: numpy array of state values
        ports: dict of port values (inputs from other components)
        meta: dict of component parameters
    
    Returns:
        x_dot: numpy array of state derivatives
    """
    # Extract states
    state1, state2, ... = x
    
    # Extract ports
    input1 = ports.get('input1', default_value)
    input2 = ports.get('input2', default_value)
    
    # Extract parameters
    param1 = meta['param1']
    param2 = meta['param2']
    
    # Compute derivatives
    x_dot = np.zeros(len(x))
    x_dot[0] = ...  # d(state1)/dt
    x_dot[1] = ...  # d(state2)/dt
    
    return x_dot
```

### 2. Output Function (Optional)

Computes outputs that other components need.

```python
def component_output(x, ports, meta):
    """
    Compute component output.
    
    Args:
        x: numpy array of states
        ports: dict of port values
        meta: dict of parameters
    
    Returns:
        output value (scalar or array)
    """
    # Extract relevant states
    state1, state2 = x[0], x[1]
    
    # Compute output
    output = ...  # function of states and parameters
    
    return output
```

### 3. Builder Function

Creates the DynamicsCore instance and sets component attributes.

```python
def build_component_core(comp_data, **kwargs):
    """
    Build component as DynamicsCore.
    
    Args:
        comp_data: dict with component parameters from JSON
        **kwargs: additional arguments (e.g., S_system for scaling)
    
    Returns:
        core: DynamicsCore object
        metadata: dict with component info
    """
    # Create core with dynamics function
    core = DynamicsCore(
        label=f'ComponentName_{comp_data["idx"]}',
        dynamics_fn=component_dynamics
    )
    
    # Extract and process parameters
    param1 = comp_data.get('param1', default_value)
    param2 = comp_data['param2']  # Required parameter
    
    # Define symbolic states
    state1, state2 = core.symbols(['state1', 'state2'])
    
    # Define Hamiltonian (energy function)
    H = 0.5 * state1**2 + 0.5 * state2**2
    
    # Add storages (states with energy)
    core.add_storages([state1, state2], H)
    
    # Add dissipations (if any)
    w = core.symbols('w')
    z = w / time_constant
    core.add_dissipations(w, z)
    
    # Add ports (inputs/outputs)
    input_port = core.symbols('input_port')
    output_port = core.symbols('output_port')
    core.add_ports([input_port], [output_port])
    
    # Store metadata
    metadata = {
        'idx': comp_data['idx'],
        'param1': param1,
        'param2': param2,
        # ... other parameters
    }
    
    core.set_metadata(metadata)
    
    # Set component interface attributes (REQUIRED)
    core.n_states = 2  # Number of states
    core.output_fn = component_output  # Output function (or None if not needed)
    core.component_type = "category"  # e.g., "generator", "exciter", "governor"
    core.model_name = "MODEL_NAME"  # e.g., "GENROU", "ESST3A"
    
    return core, metadata
```

## Component Categories

### Generators
- **Purpose**: Model synchronous generators
- **Typical states**: Rotor angle, momentum, flux linkages
- **Ports**: 
  - Inputs: `Tm` (mechanical torque), `Efd` (field voltage), `Vd`, `Vq` (network voltages)
  - Outputs: `Id`, `Iq` (currents, computed by network solution)
- **Examples**: GENROU (7 states)

### Exciters
- **Purpose**: Control generator field voltage
- **Typical states**: Voltage regulator, compensators, feedback loops
- **Ports**:
  - Inputs: `Vt` (terminal voltage), `Id`, `Iq`, `Vd`, `Vq`
  - Outputs: `Efd` (field voltage)
- **Examples**: ESST3A (5 states), EXDC2 (4 states)

### Governors
- **Purpose**: Control mechanical power/torque
- **Typical states**: Valve position, turbine states
- **Ports**:
  - Inputs: `omega` (rotor speed)
  - Outputs: `Tm` (mechanical torque)
- **Examples**: TGOV1 (2 states)

### Grid Elements
- **Purpose**: Model external grid connections
- **Typical states**: Voltage magnitude and angle
- **Ports**:
  - Inputs: Current injection
  - Outputs: Voltage phasor
- **Examples**: InfiniteBus (2 states)

## Parameter Scaling

Components may need parameter scaling for multi-base systems:

### Impedance Scaling (Generators)
```python
Z_scale = S_system / S_machine
ra_scaled = ra * Z_scale
xd_scaled = xd * Z_scale
```

### Inertia Scaling (Generators)
```python
M_scale = S_machine / S_system
M_scaled = M * M_scale
```

### Droop Scaling (Governors)
```python
R_scaled = R * (S_system / S_machine)
```

## Port Naming Conventions

Use consistent port names for interoperability:

- **Voltages**: `Vd`, `Vq` (dq-frame), `Vt` (terminal magnitude)
- **Currents**: `Id`, `Iq` (dq-frame)
- **Mechanical**: `Tm` (torque), `omega` (speed)
- **Electrical**: `Efd` (field voltage), `Te` (electrical torque)
- **Power**: `P`, `Q` (active/reactive)

## Registration

After creating a new component, register it in `utils/component_factory.py`:

```python
# In ComponentFactory.__init__()
self.model_registry = {
    # ... existing entries ...
    'YOUR_MODEL_NAME': {'category': 'generator', 'module': 'your_module_name'},
}
```

## Example: Creating a New Exciter

```python
# File: components/exciters/myexciter.py

import numpy as np
from utils.pyphs_core import DynamicsCore

def myexciter_dynamics(x, ports, meta):
    """Simple exciter dynamics"""
    vm, vr = x  # voltage measurement, regulator output
    
    Vt = ports.get('Vt', 1.0)
    Vref = meta['Vref']
    TR = meta['TR']
    KA = meta['KA']
    TA = meta['TA']
    
    x_dot = np.zeros(2)
    x_dot[0] = (Vt - vm) / TR
    x_dot[1] = (KA * (Vref - vm) - vr) / TA
    
    return x_dot

def myexciter_output(x, ports, meta):
    """Output field voltage"""
    vm, vr = x
    return vr

def build_myexciter_core(exc_data):
    core = DynamicsCore(label=f'MyExciter_{exc_data["idx"]}', 
                       dynamics_fn=myexciter_dynamics)
    
    TR = exc_data.get('TR', 0.01)
    KA = exc_data.get('KA', 200.0)
    TA = exc_data.get('TA', 0.01)
    Vref = exc_data.get('Vref', 1.0)
    
    vm, vr = core.symbols(['vm', 'vr'])
    H = 0.5 * (TR * vm**2 + TA * vr**2)
    core.add_storages([vm, vr], H)
    
    Vt_in = core.symbols('Vt_in')
    Efd_out = core.symbols('Efd_out')
    core.add_ports([Vt_in], [Efd_out])
    
    metadata = {
        'idx': exc_data['idx'],
        'syn': exc_data['syn'],
        'TR': TR,
        'KA': KA,
        'TA': TA,
        'Vref': Vref
    }
    
    core.set_metadata(metadata)
    
    # Required interface attributes
    core.n_states = 2
    core.output_fn = myexciter_output
    core.component_type = "exciter"
    core.model_name = "MYEXCITER"
    
    return core, metadata
```

Then register in `component_factory.py`:
```python
'MYEXCITER': {'category': 'exciter', 'module': 'myexciter'},
```

And use in JSON:
```json
{
  "MYEXCITER": [
    {
      "idx": 1,
      "u": 1,
      "name": "Exciter_1",
      "syn": 1,
      "TR": 0.01,
      "KA": 200.0,
      "TA": 0.01,
      "Vref": 1.0
    }
  ]
}
```

## Testing New Components

1. Create a test case JSON with your component
2. Run the system builder to verify it loads correctly
3. Run a simulation to verify dynamics are correct
4. Check state evolution and output values

## Common Pitfalls

1. **Forgetting to set component attributes**: Always set `n_states`, `output_fn`, `component_type`, and `model_name`
2. **Incorrect port names**: Use standard port names for compatibility
3. **Missing parameter validation**: Add defaults for optional parameters
4. **Forgetting registration**: New components must be registered in the factory
5. **Wrong parameter scaling**: Apply correct scaling for multi-base systems

## Best Practices

1. **Document parameters**: Add clear docstrings explaining each parameter
2. **Use meaningful defaults**: Provide sensible defaults for optional parameters
3. **Validate inputs**: Check for valid parameter ranges
4. **Test thoroughly**: Test with different parameter values and operating conditions
5. **Follow naming conventions**: Use consistent naming for states, ports, and parameters
