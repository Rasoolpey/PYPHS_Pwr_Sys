## Modular PyPHS Power System - Fault Simulation

Data-driven Port-Hamiltonian power system with fault simulation capability.

### Directory Structure
```
├── components/
│   ├── generators/genrou.py      # GENROU model
│   ├── exciters/exdc2.py         # EXDC2 model
│   ├── governors/tgov1.py        # TGOV1 model
│   ├── network/network_builder.py # Network construction
│   └── loads/                     # Future models
├── utils/
│   ├── pyphs_core.py             # Standalone PyPHS Core
│   ├── component_factory.py      # Component loader
│   ├── system_builder.py         # System assembler
│   ├── fault_simulator.py        # Fault simulation
│   └── model_templates.py        # Templates
├── test_cases/
│   └── Kundur_System/
│       └── kundur_full.json      # System configuration
├── main.py                        # Basic example
├── demo.py                        # Demonstrations
├── test_system.py                 # Component tests
├── test_fault.py                  # Fault simulation test
└── working_reference.py           # Original working code
```

### Quick Start

**Basic system build:**
```python
from utils.system_builder import PowerSystemBuilder

builder = PowerSystemBuilder('test_cases/Kundur_System/kundur_full.json')
builder.build_all_components()
builder.summary()
```

**Fault simulation:**
```python
from utils.fault_simulator import ModularFaultSimulator

sim = ModularFaultSimulator('test_cases/Kundur_System/kundur_full.json')
x0 = sim.initialize_equilibrium()
sol = sim.simulate(x0, t_end=15.0)
sim.plot_results(sol)
```

**Run tests:**
```bash
python main.py              # Build system
python demo.py              # Demonstrations
python test_system.py       # Component tests
python test_fault.py        # Fault simulation
```

### Features
- **Data-driven**: All parameters from JSON
- **Modular**: Easy to add new component models
- **Fault simulation**: 3-phase fault with Ybus switching
- **No PyPHS dependency**: Standalone implementation
- **Working dynamics**: Uses proven Port-Hamiltonian structure

### Adding New Models
1. Create file: `components/generators/newmodel.py`
2. Implement: `build_newmodel_core(gen_data, S_system)`
3. Register: `factory.register_model('generators', 'NEWMODEL', 'newmodel')`
4. Add to JSON under `"NEWMODEL": [...]`

### Adding New Test Cases
Create new directory under `test_cases/YourSystem/` with JSON configuration:
```bash
mkdir test_cases/MySystem
cp test_cases/Kundur_System/kundur_full.json test_cases/MySystem/mysystem.json
# Edit mysystem.json with your parameters
python main.py test_cases/MySystem/mysystem.json
```

See `USAGE.py` and `QUICKREF.py` for details.
