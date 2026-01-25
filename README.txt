# Modular Port-Hamiltonian Power System Framework

A modular and extensible Port-Hamiltonian (PHS) modeling framework for power system dynamics simulation and analysis.

## Overview

This framework provides a **fully modular architecture** for Port-Hamiltonian modeling of power systems, enabling dynamic system reconfiguration, component reusability, and rapid prototyping of new analysis tools. The modular design allows users to:

- **Define system models via JSON files** in the `test_cases/` folder - no code changes required
- **Create reusable component libraries** in the `components/` folder (generators, exciters, governors, network elements)
- **Extend functionality easily** by adding new analysis modules to the `utils/` folder
- **Add/remove system components dynamically** leveraging the Port-Hamiltonian structure

## Key Features

### 🔧 **Modular Component Design**
- Each power system component (generator, exciter, governor) is defined independently
- Components can be reused across different system configurations
- New models can be added without modifying existing code
- Component models defined in `components/` subfolders with clear separation by type

### 📋 **JSON-Based System Configuration**
- Complete system parameters defined in JSON files under `test_cases/`
- Change system topology, parameters, or models by editing JSON - no coding required
- Support for multiple test cases and system scenarios
- Easy to version control and share system configurations

### 🧩 **Expandable Architecture**
- Port-Hamiltonian structure enables plug-and-play component addition/removal
- New analysis tools can be added to `utils/` folder (e.g., fault simulation, impedance scanning)
- Clean separation between physics models, system building, and analysis functions
- Framework designed for continuous expansion

### 🚀 **Current Analysis Capabilities**
- **Fault Simulation** (`fault_sim_modular.py`): Configurable fault analysis with dynamic Ybus switching
- **Impedance Scanner** (`impedance_scanner.py`): Frequency-domain impedance analysis at any bus
- **Load Flow**: Network solution with Park transformations
- **Time-Domain Simulation**: Full nonlinear dynamics with adaptive solvers

### 🔬 **Port-Hamiltonian Benefits**
- Energy-based modeling ensures physical consistency
- Natural framework for component interconnection
- Stability analysis through energy functions
- Easy to add/remove components while preserving energy balance

### Directory Structure
```
├── components/                    # Component Model Library (Expandable)
│   ├── generators/               # Generator models (GENROU, etc.)
│   │   └── genrou.py            # 6th order synchronous machine model
│   ├── exciters/                # Excitation system models
│   │   └── exdc2.py             # DC exciter type 2
│   ├── governors/               # Governor models
│   │   └── tgov1.py             # Steam turbine governor
│   ├── network/                 # Network elements
│   │   └── network_builder.py  # Ybus construction and network analysis
│   └── [future models]          # Add new component types here (loads, FACTS, etc.)
│
├── utils/                        # System Building and Analysis Tools
│   ├── pyphs_core.py            # Standalone Port-Hamiltonian Core (symbolic math)
│   ├── component_factory.py     # Dynamic component loading and registry
│   ├── system_builder.py        # JSON parser and system assembler
│   ├── system_coordinator.py    # Network solver, Ybus management, Park transforms
│   ├── fault_sim_modular.py     # ⭐ Fault simulation module (configurable faults)
│   ├── impedance_scanner.py     # ⭐ Frequency-domain impedance analysis
│   └── model_templates.py       # Component templates for rapid development
│
├── test_cases/                   # System Configuration Files (JSON)
│   ├── Kundur_System/           # Example: Kundur's 4-machine system
│   │   ├── kundur_full.json     # Complete system definition
│   │   └── system_config.json   # Alternative configuration
│   └── [Your_System]/           # Add your own test cases here
│
├── outputs/                      # Generated plots and results
│
├── main.py                       # Basic system building example
├── test_system.py               # Component verification tests
├── test_modular_fault.py        # Fault simulation tests
└── test_impedance_scanning.py   # Impedance scanning tests
```

## Quick Start

### 1. Build System from JSON Configuration

```python
from utils.system_builder import PowerSystemBuilder

# Load system from JSON file
builder = PowerSystemBuilder('test_cases/Kundur_System/kundur_full.json')
builder.build_all_components()
builder.summary()
```

### 2. Run Fault Simulation

```python
from utils.fault_sim_modular import ModularFaultSimulator

# Initialize simulator
sim = ModularFaultSimulator('test_cases/Kundur_System/kundur_full.json')

# Configure fault (bus index, impedance, start time, duration)
sim.set_fault(bus_idx=1, impedance=0.01j, start_time=2.0, duration=0.15)

# Run simulation
x0 = sim.initialize_equilibrium()
sol = sim.simulate(x0, t_end=15.0)
sim.plot_results(sol)
```

### 3. Perform Impedance Scanning

```python
from utils.impedance_scanner import ImpedanceScanner
import numpy as np

# Initialize scanner
scanner = ImpedanceScanner('test_cases/Kundur_System/kundur_full.json')

# Define frequency range
freqs = np.logspace(-1, 2, 100)  # 0.1 Hz to 100 Hz

# Scan impedance at specific bus
f, Z_mag, Z_phase, Z_dq = scanner.scan_impedance(bus_idx=0, freqs=freqs)
```

### 4. Run Test Scripts

```bash
python main.py                      # Build and verify system
python test_system.py               # Test individual components
python test_modular_fault.py        # Test fault simulation
python test_impedance_scanning.py   # Test impedance analysis
```

## How to Extend the Framework

### Adding New Component Models

New controllers and dynamic models can be defined in the `components/` folder under the appropriate subfolder. These models become immediately reusable across all system configurations.

**Example: Add a new generator model**

1. **Create model file**: `components/generators/genroe.py`
   ```python
   def build_genroe_core(gen_data, S_system):
       """Build 8th order generator model"""
       # Define your Port-Hamiltonian structure here
       # Return PyPHS core object
       pass
   ```

2. **Register in factory**: Update `utils/component_factory.py`
   ```python
   factory.register_model('generators', 'GENROE', 'genroe')
   ```

3. **Use in any system**: Just reference in your JSON file
   ```json
   {
     "generators": {
       "GENROE": [
         {"bus": 1, "parameters": {...}}
       ]
     }
   }
   ```

**Example: Add a new exciter or governor**
- Create file: `components/exciters/sexs.py` or `components/governors/hygov.py`
- Follow the same registration pattern
- Models are immediately available for all systems

### Adding New System Configurations

Create new test cases by defining JSON files in `test_cases/` folder:

1. **Create directory**:
   ```bash
   mkdir test_cases/MySystem
   ```

2. **Define JSON configuration**: `test_cases/MySystem/system_config.json`
   ```json
   {
     "system": {
       "S_base": 100.0,
       "f_base": 60.0
     },
     "generators": {
       "GENROU": [...]
     },
     "exciters": {
       "EXDC2": [...]
     },
     "network": {...}
   }
   ```

3. **Run with your configuration**:
   ```python
   builder = PowerSystemBuilder('test_cases/MySystem/system_config.json')
   ```

No code changes required - just define parameters in JSON!

### Adding New Analysis Modules

The framework is designed for easy extension with new functionality. Recent additions include fault simulation and impedance scanning - you can add more!

**Example: Add a new analysis tool**

1. **Create module**: `utils/your_new_module.py`
   ```python
   from utils.system_builder import PowerSystemBuilder
   from utils.system_coordinator import PowerSystemCoordinator
   
   class YourAnalysisTool:
       def __init__(self, json_file):
           self.builder = PowerSystemBuilder(json_file)
           self.builder.build_all_components()
           self.coordinator = PowerSystemCoordinator(self.builder)
       
       def your_analysis_method(self):
           # Implement your analysis using the built system
           pass
   ```

2. **Create test script**: `test_your_module.py`
   ```python
   from utils.your_new_module import YourAnalysisTool
   
   tool = YourAnalysisTool('test_cases/Kundur_System/kundur_full.json')
   results = tool.your_analysis_method()
   ```

**Possible extensions**:
- Small-signal stability analysis
- Modal analysis and participation factors
- Optimal power flow
- State estimation
- Voltage stability analysis
- Transient stability screening
- Custom control strategies

The Port-Hamiltonian framework and modular architecture make it easy to add any new function!

## Module Architecture and Responsibilities

The framework uses a clean separation of concerns with each module handling specific tasks:

### Core Modules (`utils/`)

**pyphs_core.py** - Port-Hamiltonian Engine
- Standalone symbolic Port-Hamiltonian implementation
- No external PyPHS dependencies
- Manages Hamiltonian storage, symbolic mathematics, and energy functions

**component_factory.py** - Component Registry
- Dynamic component loading system
- Model registration and lookup
- Enables plug-and-play component architecture

**system_builder.py** - System Assembler
- Parses JSON configuration files
- Builds all system components (generators, exciters, governors, network)
- Stores component metadata for system-wide access
- Creates the complete system model from configuration

**system_coordinator.py** - Network Solver
- Manages Ybus matrices (pre-fault and during-fault)
- Network solution with Park transformations (dq0 ↔ abc)
- Fault application: `apply_fault(bus_idx, impedance)`
- Coordinates interaction between components and network

**fault_sim_modular.py** ⭐ - Fault Simulation Module
- Configurable fault scenarios (any bus, any impedance, any time)
- Automatic Ybus switching during fault events
- Time-domain simulation with adaptive integration
- Post-simulation plotting and analysis
- Example: `sim.set_fault(bus_idx=1, impedance=0.01j, start_time=2.0, duration=0.15)`

**impedance_scanner.py** ⭐ - Impedance Analysis Module
- Frequency-domain impedance scanning at any system bus
- Linearizes Port-Hamiltonian dynamics around operating point
- State-space model extraction (A, B, C, D matrices)
- Transfer function analysis: Z(s) = C(sI-A)^(-1)B + D
- Generates impedance Bode plots (magnitude and phase)

**model_templates.py** - Development Templates
- Boilerplate code for creating new component models
- Standardized interfaces for component integration
- Speeds up development of new models

### Component Models (`components/`)

All component models follow a consistent Port-Hamiltonian structure and can be mixed and matched across different system configurations.

**generators/** - Generator Models
- `genrou.py`: 6th order round-rotor synchronous machine (industry standard)
- [Expandable]: Add GENSAL, GENSAE, custom models

**exciters/** - Excitation Systems
- `exdc2.py`: DC exciter type 2 (IEEE standard)
- [Expandable]: Add SEXS, IEEET1, custom exciters

**governors/** - Governor Models  
- `tgov1.py`: Steam turbine governor type 1
- [Expandable]: Add HYGOV, GGOV1, custom governors

**network/** - Network Elements
- `network_builder.py`: Ybus construction, admittance calculations
- [Expandable]: Add transformer models, FACTS devices, etc.

### Test Cases (`test_cases/`)

JSON-based system definitions that can be loaded without code modification:

**Kundur_System/** - Four Machine Two Area System
- Classic benchmark system for stability studies
- `kundur_full.json`: Complete parameter set
- Includes generators, exciters, governors, network, and loads

## Example Workflows

### Workflow 1: Fault Analysis on Different Systems

```python
from utils.fault_sim_modular import ModularFaultSimulator

# Analyze same fault on different system configurations
for system in ['kundur_full.json', 'system_config.json']:
    sim = ModularFaultSimulator(f'test_cases/Kundur_System/{system}')
    sim.set_fault(bus_idx=1, impedance=0.01j, start_time=2.0, duration=0.15)
    
    x0 = sim.initialize_equilibrium()
    sol = sim.simulate(x0, t_end=15.0)
    sim.plot_results(sol)
```

### Workflow 2: Multi-Bus Impedance Analysis

```python
from utils.impedance_scanner import ImpedanceScanner
import numpy as np

scanner = ImpedanceScanner('test_cases/Kundur_System/kundur_full.json')
buses = scanner.list_buses()
freqs = np.logspace(-1, 2, 100)

# Scan all buses
for bus in buses:
    f, Z_mag, Z_phase, Z_dq = scanner.scan_impedance(bus['idx'], freqs)
    # Compare impedance characteristics across buses
```

### Workflow 3: Component Reusability

```python
# Use the same GENROU model in different systems
# System 1: test_cases/System_A/config.json
# System 2: test_cases/System_B/config.json
# Both reference components/generators/genrou.py

from utils.system_builder import PowerSystemBuilder

builder_A = PowerSystemBuilder('test_cases/System_A/config.json')
builder_A.build_all_components()

builder_B = PowerSystemBuilder('test_cases/System_B/config.json')
builder_B.build_all_components()

# Same component code, different configurations!
```

## Advantages of Port-Hamiltonian Formulation

### Energy-Based Modeling
- Natural representation of power system energy storage and dissipation
- Physical consistency guaranteed by Hamiltonian structure
- Clear identification of energy functions (kinetic, magnetic, etc.)

### Component Interconnection
- Port-based coupling between components
- Power-preserving interconnection
- Easy to add/remove components while maintaining energy balance

### Stability Analysis
- Passivity and stability analysis through energy functions
- Lyapunov stability certificates
- Energy-based control design

### Modular Structure
- Each component has well-defined input/output ports
- Natural framework for modular system assembly
- Facilitates parallel development of components

## Technical Details

### State Vector Organization
For a 4-machine system:
- Each generator: 13 states (7 electrical + 4 exciter + 2 governor)
- Total: 52 states for complete system dynamics

### Network Solution
- Admittance matrix (Ybus) formulation
- Park transformation for dq0 reference frame
- Iterative solution for nonlinear current-voltage relationships

### Fault Modeling
- Ybus modification during fault
- Automatic switching at fault initiation and clearing
- Preserves energy balance through fault transition

## Future Extensions

The modular architecture enables easy addition of:
- **Load models**: Static, dynamic, motor loads
- **FACTS devices**: SVC, STATCOM, UPFC
- **Renewable integration**: Wind turbines, solar PV with inverters
- **Protection systems**: Relay models, breaker logic
- **Advanced analysis**: Eigenvalue analysis, participation factors, mode shapes
- **Optimization tools**: Optimal power flow, control tuning
- **Real-time simulation**: Hardware-in-the-loop capabilities

## Contributing

To contribute new models or analysis tools:
1. Follow the Port-Hamiltonian structure conventions
2. Use the template in `model_templates.py`
3. Add comprehensive documentation
4. Create test cases demonstrating functionality
5. Update this README with new features

## References

- Port-Hamiltonian modeling of electrical networks
- Kundur's "Power System Stability and Control"
- IEEE standard models for generators, exciters, and governors

---

**Framework Version**: 1.0  
**Author**: [Your Name]  
**Last Updated**: January 2026
