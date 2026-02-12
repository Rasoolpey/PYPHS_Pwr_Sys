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
- New analysis tools can be added to `utils/` folder
- Clean separation between physics models, system building, and analysis functions
- Framework designed for continuous expansion

### ⚡ **Performance Optimization (Numba JIT + LU Caching)**
- **Numba JIT compilation** for all component dynamics functions (~10-50x faster per call)
  - Array-based function signatures replace Python dicts (Numba `@njit` compatible)
  - Pre-allocated port buffers eliminate per-call allocation overhead
  - Cached compilation (`cache=True`) avoids recompilation across sessions
- **LU factorization caching** for network solver (~6x overall speedup)
  - Y_aug matrix factored once per fault transition, `lu_solve` per iteration
  - Cache key: `(use_fault, fault_bus, fault_impedance)` — only recomputed on topology changes
- **Combined speedup**: ~6x (Kundur 4-machine: 53s→9s, IEEE 14-bus: 33s→5s)
- JIT warmup before simulation start eliminates first-call compilation penalty

### 🚀 **Current Analysis Capabilities**
- **Fault Simulation**: Configurable fault analysis with dynamic Ybus switching
- **Lyapunov Stability Analysis**: Energy-based stability assessment
  - Linearized stability via eigenvalue analysis
  - Port-Hamiltonian energy function as Lyapunov candidate
  - Region of attraction estimation with dV/dt criterion
  - Transient energy margin computation
  - Multi-panel stability visualization
- **Impedance Scanning** (Three complementary methods):
  - Frequency-domain linearization (fast, small-signal)
  - IMTB multisine injection (MIMO, medium-amplitude)
  - White noise time-domain (large-signal, nonlinear)
- **Load Flow**: Network solution with Park transformations
- **Time-Domain Simulation**: Full nonlinear dynamics with adaptive solvers

### 🔬 **Port-Hamiltonian Benefits**
- Energy-based modeling ensures physical consistency
- Natural framework for component interconnection
- Stability analysis through energy functions
- Easy to add/remove components while preserving energy balance

## Directory Structure

```
├── components/                    # Component Model Library (Expandable)
│   ├── generators/               # Generator models (GENROU, etc.)
│   │   └── genrou.py            # 6th order synchronous machine model
│   ├── exciters/                # Excitation system models
│   │   ├── exdc2.py             # DC exciter type 2
│   │   ├── exst1.py             # IEEE ST1 static exciter
│   │   └── esst3a.py            # IEEE ST3A static exciter
│   ├── governors/               # Governor models
│   │   ├── tgov1.py             # Steam turbine governor
│   │   └── ieeeg1.py            # IEEE G1 multi-stage steam turbine
│   ├── network/                 # Network elements
│   │   └── network_builder.py  # Ybus construction and network analysis
│   └── [future models]          # Add new component types here (loads, FACTS, etc.)
│
├── utils/                        # System Building and Analysis Tools
│   ├── pyphs_core.py            # Standalone Port-Hamiltonian Core (symbolic math)
│   ├── component_factory.py     # Dynamic component loading and registry
│   ├── system_builder.py        # JSON parser and system assembler
│   ├── system_coordinator.py    # Network solver, Ybus management, LU caching
│   ├── fault_sim_modular.py     # ⭐ Fault simulation module (JIT-accelerated)
│   ├── numba_kernels.py         # ⚡ JIT registry, prepare_jit_data, warmup
│   ├── power_flow.py            # Newton-Raphson AC power flow solver
│   ├── lyapunov_analyzer.py     # ⭐ Lyapunov stability analysis
│   ├── impedance_scanner.py     # ⭐ Frequency-domain impedance
│   ├── imtb_scanner.py          # ⭐ IMTB multisine impedance scanner
│   ├── impedance_scanner_td.py  # ⭐ Time-domain white noise impedance scanner
│   ├── im_analysis_lib.py       # Impedance analysis utilities and plotting
│   └── model_templates.py       # Component templates for rapid development
│
├── test_cases/                   # System Configuration Files (JSON)
│   ├── Kundur_System/           # Kundur's 4-machine, 2-area system (GENROU + EXDC2 + TGOV1)
│   │   ├── kundur_full.json     # Complete system definition
│   │   ├── system_config.json   # Alternative configuration
│   │   └── EXAMPLE.md           # Detailed example documentation
│   ├── ieee14bus/               # IEEE 14-bus system (5 generators, mixed exciters/governors)
│   │   └── ieee14_system.json   # GENROU + ESST3A/EXST1 + IEEEG1/TGOV1
│   ├── ieee39bus/               # IEEE 39-bus (New England) system (10 generators)
│   │   └── ieee39_system.json   # GENROU + ESST3A + IEEEG1
│   ├── Thevenin_model/          # Single-machine infinite-bus (SMIB) system
│   │   └── thevenin_system.json # GENROU + EXDC2 + TGOV1
│   └── [Your_System]/           # Add your own test cases here
│
├── outputs/                      # Generated plots and results
│
├── main.py                       # Basic system building example
├── test_system.py               # Component verification tests
├── test_modular_fault.py        # Fault simulation tests
├── test_lyapunov.py             # Lyapunov stability analysis tests
├── test_impedance_scanning.py   # Frequency-domain impedance scanning tests
├── test_imtb_scanning.py        # IMTB multisine impedance tests
└── test_impedance_td.py         # Time-domain white noise impedance tests
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

**Output:**
```
Building power system components...
  Building 4 GENROU generators...
  Building 4 EXDC2 exciters...
  Building 4 TGOV1 governors...
  Building network from Line data...
Component build complete!
```

### 2. Run Fault Simulation

```python
from utils.fault_sim_modular import ModularFaultSimulator

# Initialize simulator with system and simulation configs
sim = ModularFaultSimulator(
    system_json='test_cases/Kundur_System/kundur_full.json',
    simulation_json='test_cases/Kundur_System/simulation_fault.json'
)

# Alternatively, configure fault programmatically
sim.set_fault(bus_idx=1, impedance=0.01j, start_time=2.0, duration=0.15)

# Initialize equilibrium (power flow runs automatically if configured in JSON)
x0 = sim.initialize_equilibrium()

# Run simulation
sol = sim.simulate(x0, t_end=15.0)
sim.plot_results(sol)
```

**Simulation JSON Example** (⚠️ **IMPORTANT**: Always enable `run_power_flow` for machine-precision initialization):
```json
{
  "time": {"t_end": 15.0, "n_points": 3000},
  "fault": {"enabled": true, "bus": 1, "impedance": "0.01j", 
            "start_time": 2.0, "duration": 0.15},
  "solver": {"method": "Radau", "rtol": 1e-6, "atol": 1e-8},
  "initialization": {"run_power_flow": true, "power_flow_verbose": true}
}
```

**Note**: The `run_power_flow: true` setting is **required** for perfect initialization (Max |dx/dt| < 1e-12). Without it, the system will drift and show large transients at startup.
```

### 3. Analyze Lyapunov Stability

```python
from utils.lyapunov_analyzer import LyapunovStabilityAnalyzer

# Initialize analyzer
analyzer = LyapunovStabilityAnalyzer('test_cases/Kundur_System/kundur_full.json')

# Initialize equilibrium point
x_eq = analyzer.initialize_equilibrium()

# Perform linearized stability analysis
results = analyzer.analyze_linearized_stability()
print(f"System is: {'STABLE' if results['is_stable'] else 'UNSTABLE'}")
print(f"Unstable modes: {results['unstable_count']}")

# Estimate region of attraction
region_results = analyzer.estimate_stability_region(n_samples=500)
print(f"Estimated critical energy: {region_results['V_critical']:.4f}")

# Generate comprehensive report
analyzer.generate_stability_report('outputs/stability_report.txt')
```

### 4. Perform Impedance Scanning

**Method 1: Frequency-Domain Linearization (Fast)**
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

**Method 2: IMTB Multisine (MIMO)**
```python
from utils.imtb_scanner import IMTBScanner

scanner = IMTBScanner('test_cases/Kundur_System/kundur_full.json')

# Run MIMO scan with multisine injection
freqs, Z_dq_matrices = scanner.run_mimo_scan(
    bus_idx=2, 
    freqs=np.logspace(-1, 2, 30),
    amplitude=0.10
)
```

**Method 3: Time-Domain White Noise (Nonlinear)**
```python
from utils.impedance_scanner_td import ImpedanceScannerTD

scanner = ImpedanceScannerTD('test_cases/Kundur_System/kundur_full.json')

# Run white noise scan
sol, bus_idx = scanner.run_scan(
    bus_idx=2, 
    f_max=50.0,
    amplitude=0.01,
    duration=60.0
)

# Post-process to get impedance
freqs, Z_est = scanner.post_process_tfe(sol, bus_idx)
```

### 5. Run Test Scripts

```bash
python main.py                      # Build and verify system
python test_system.py               # Test individual components
python test_modular_fault.py        # Test fault simulation (Kundur system)
python test_ieee14bus_nofault.py    # IEEE 14-bus equilibrium verification
python test_ieee14bus_fault.py      # IEEE 14-bus fault simulation
python test_ieee39bus_nofault.py    # IEEE 39-bus equilibrium verification
python test_ieee39bus_fault.py      # IEEE 39-bus fault simulation
python test_lyapunov.py             # Test Lyapunov stability analysis
python test_impedance_scanning.py   # Test frequency-domain impedance
python test_imtb_scanning.py        # Test IMTB multisine impedance
python test_impedance_td.py         # Test time-domain impedance
```

## Example System

📖 **For a complete worked example**, see the [Kundur System Documentation](test_cases/Kundur_System/EXAMPLE.md)

The Kundur two-area four-machine system is a classic benchmark that demonstrates:
- Multi-machine system configuration
- Inter-area oscillations
- Fault response and stability
- All analysis capabilities of the framework

The example documentation includes:
- Complete system parameters
- Power flow results
- Test cases and expected outputs
- Verification results
- Troubleshooting guide

## How to Extend the Framework

### Adding New Component Models

New controllers and dynamic models can be defined in the `components/` folder under the appropriate subfolder. These models become immediately reusable across all system configurations.

**Example: Add a new generator model**

1. **Create model file**: `components/generators/genroe.py`
   ```python
   from utils.pyphs_core import DynamicsCore
   import sympy as sp
   
   def genroe_dynamics(x, ports, meta):
       """8th order generator model dynamics"""
       # Extract states
       delta, p, psi_d, psi_q, psi_f, psi_kd, psi_kq, psi_kq2 = x
       
       # Extract ports
       Id, Iq = ports['Id'], ports['Iq']
       Tm, Efd = ports['Tm'], ports['Efd']
       
       # Implement 8th order differential equations
       # ...
       return x_dot
   
   def build_genroe_core(gen_data, S_system):
       """Build 8th order generator PyPHS core"""
       # Create DynamicsCore
       core = DynamicsCore()
       core.set_dynamics_function(genroe_dynamics)
       
       # Set metadata
       metadata = {
           'xd': gen_data['parameters']['xd'],
           # ... other parameters
       }
       core.set_metadata(metadata)
       
       return core, metadata
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
         {"bus": 1, "Sn": 900.0, "parameters": {...}}
       ]
     }
   }
   ```

**Example: Add a new exciter or governor**
- Create file: `components/exciters/sexs.py` or `components/governors/hygov.py`
- Follow the same DynamicsCore pattern
- Register in component factory
- Models are immediately available for all systems

### Adding New System Configurations

Create new test cases by defining JSON files in `test_cases/` folder:

1. **Create directory**:
   ```bash
   mkdir test_cases/MySystem
   ```

2. **Define system JSON**: `test_cases/MySystem/system_config.json`
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
     "governors": {
       "TGOV1": [...]
     },
     "network": {
       "Bus": [...],
       "Line": [...],
       "Slack": [...],
       "PV": [...]
     }
   }
   ```

3. **Define simulation JSON** (REQUIRED): `test_cases/MySystem/simulation.json`
   ```json
   {
     "time": {"t_end": 15.0, "n_points": 3000},
     "fault": {"enabled": false},
     "solver": {"method": "Radau", "rtol": 1e-6, "atol": 1e-8},
     "initialization": {
       "run_power_flow": true,
       "power_flow_verbose": true
     }
   }
   ```

   **⚠️ Critical**: Always include `"run_power_flow": true` in initialization to achieve machine-precision equilibrium. Without it, the system will drift and show large transients.

4. **Run with your configuration**:
   ```python
   builder = PowerSystemBuilder('test_cases/MySystem/system_config.json')
   ```

No code changes required - just define parameters in JSON!

### Adding New Analysis Modules

The framework is designed for easy extension with new functionality.

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
           # Access system components
           n_gen = len(self.builder.generators)
           
           # Use coordinator for network solution
           gen_states = self.get_equilibrium_states()
           Id, Iq, Vd, Vq = self.coordinator.solve_network(gen_states)
           
           # Perform your analysis
           results = self.compute_something()
           return results
   ```

2. **Create test script**: `test_your_module.py`
   ```python
   from utils.your_new_module import YourAnalysisTool
   
   tool = YourAnalysisTool('test_cases/Kundur_System/kundur_full.json')
   results = tool.your_analysis_method()
   ```

**Possible extensions**:
- ✅ Small-signal stability analysis (implemented)
- ✅ Transient stability screening (implemented)
- Modal analysis and participation factors
- Optimal power flow
- State estimation
- Voltage stability analysis
- Custom control strategies
- Machine learning integration

The Port-Hamiltonian framework and modular architecture make it easy to add any new function!

## Module Architecture and Responsibilities

The framework uses a clean separation of concerns with each module handling specific tasks:

### Core Modules (`utils/`)

**pyphs_core.py** - Port-Hamiltonian Engine
- Standalone symbolic Port-Hamiltonian implementation
- No external PyPHS dependencies
- Manages Hamiltonian storage, symbolic mathematics, and energy functions
- **DynamicsCore class**: Extends Core with numerical dynamics capability
- Each component provides dynamics function: `dynamics(x, ports, metadata)`

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
- Builds Ybus matrix dynamically from Line data
- Manages pre-fault and during-fault admittance matrices
- **Constant power load model** with iterative voltage solution
- Park transformations (machine dq ↔ system RI frame)
- Fault application: `apply_fault(bus_idx, impedance)`
- Coordinates generator-network interface

**fault_sim_modular.py** - Fault Simulation Module
- **Fully dynamic** - adapts to any number of generators from JSON
- Direct equilibrium initialization from power flow data
- Configurable fault scenarios (any bus, any impedance, any time)
- Automatic Ybus switching during fault events
- Time-domain simulation with adaptive integration
- Post-simulation plotting and analysis

**lyapunov_analyzer.py** - Lyapunov Stability Analyzer
- **Port-Hamiltonian energy-based stability analysis**
- Hamiltonian as natural Lyapunov function candidate
- Linearized stability: Jacobian eigenvalue analysis
- **Region of Attraction**: Estimates stability boundary using dV/dt < 0
- **Transient Energy Margin**: Computes energy absorbed during faults
- **Passivity Verification**: Checks energy dissipation
- **COI Frame**: Removes rotational invariance for multi-machine systems
- Comprehensive visualization: eigenvalues, phase portraits, energy landscapes
- Performance: 500 sample estimation in ~80 seconds (10-20x faster)

**Impedance Scanners** (Three complementary methods)
- `impedance_scanner.py`: Frequency-domain linearization (analytical, fast)
- `imtb_scanner.py`: IMTB multisine (MIMO, medium amplitude)
- `impedance_scanner_td.py`: White noise time-domain (full nonlinear)

**im_analysis_lib.py** - Impedance Analysis Utilities
- Plotting functions for impedance data
- Analysis helpers for resonance detection
- Stability margin calculations

**model_templates.py** - Development Templates
- Boilerplate code for creating new component models
- Standardized interfaces for component integration

### Component Models (`components/`)

All component models follow a consistent Port-Hamiltonian structure and can be mixed and matched across different system configurations.

**generators/** - Generator Models
- `genrou.py`: 6th order round-rotor synchronous machine (IEEE standard)
- [Expandable]: Add GENSAL, GENSAE, custom models

**exciters/** - Excitation Systems
- `exdc2.py`: DC exciter type 2 (IEEE standard)
- `exst1.py`: Static exciter type 1 (IEEE ST1) with lead-lag and washout feedback
- `esst3a.py`: Static exciter type ST3A (IEEE ESST3A) with VE compensation, rectifier model, inner/outer regulators
- [Expandable]: Add SEXS, IEEET1, custom exciters

**governors/** - Governor Models
- `tgov1.py`: Steam turbine governor type 1
- `ieeeg1.py`: IEEE G1 multi-stage steam turbine governor with 4 turbine/reheater stages and rate-limited valve control
- [Expandable]: Add HYGOV, GGOV1, custom governors

**network/** - Network Elements
- `network_builder.py`: Ybus construction, admittance calculations
- [Expandable]: Add transformer models, FACTS devices

## Analysis Methods Comparison

### Impedance Scanning Methods

| Feature | Frequency-Domain | IMTB Multisine | Time-Domain White Noise |
|---------|-----------------|----------------|------------------------|
| **Technique** | Linearization | Multisine DFT | Trajectory Difference + Welch |
| **Speed** | Very Fast (~seconds) | Medium (~minutes) | Slow (~5-15 minutes) |
| **Frequency Points** | 100+ points | 20-50 points | Resolution = duration⁻¹ |
| **Amplitude** | Infinitesimal (ε=10⁻⁵) | 0.01-0.10 pu | 0.001-0.10 pu |
| **Nonlinearity** | ❌ Linear only | ✓ Partial (saturation) | ✓ Full nonlinear |
| **MIMO** | ✓ 2×2 dq matrix | ✓ 2×2 dq matrix | Single complex Z |
| **Best For** | Initial design | Controller tuning | Validation testing |

**Choosing the Right Method:**
- **For initial design**: Use Frequency-Domain (fast, clean, parametric studies)
- **For controller tuning**: Use IMTB (medium amplitude, precise frequencies, MIMO)
- **For validation**: Use Time-Domain (realistic, captures nonlinearity)
- **For complete analysis**: Use all three and compare results

### Lyapunov Stability Analysis

**Methods:**
1. **Linearized Stability**: Eigenvalue analysis at equilibrium
   - Fast (seconds)
   - Local analysis
   - Identifies unstable, stable, and marginal modes

2. **Region of Attraction**: Lyapunov function level sets
   - Medium speed (~80s for 500 samples)
   - Global analysis
   - Estimates stability boundary

3. **Transient Energy Margin**: Energy absorbed during faults
   - Fast (< 1 second)
   - Practical stability assessment
   - Compares fault energy to critical energy

4. **Passivity Verification**: Energy evolution along trajectories
   - Requires simulation (~seconds)
   - Verifies V(t) decreasing
   - Confirms dissipative behavior

## Advantages of Port-Hamiltonian Formulation

### Energy-Based Modeling
- Natural representation of power system energy storage and dissipation
- Physical consistency guaranteed by Hamiltonian structure
- Clear identification of energy functions (kinetic, magnetic, potential)

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

## Performance Optimization

### Numba JIT Compilation

All 7 component dynamics functions have been converted to Numba `@njit(cache=True)` compiled versions using flat numpy arrays instead of Python dictionaries:

| Component | Dict-based (original) | JIT-compiled (new) | States | Meta Size |
|-----------|----------------------|---------------------|--------|-----------|
| GENROU    | `genrou_dynamics(x, ports, meta)` | `genrou_dynamics_jit(x, ports, meta)` | 7 | 8 |
| EXDC2     | `exdc2_dynamics(x, ports, meta)` | `exdc2_dynamics_jit(x, ports, meta)` | 4 | 13 |
| EXST1     | `exst1_dynamics(x, ports, meta)` | `exst1_dynamics_jit(x, ports, meta)` | 4 | 13 |
| ESST3A    | `esst3a_dynamics(x, ports, meta)` | `esst3a_dynamics_jit(x, ports, meta)` | 5 | 24 |
| IEEEX1    | `ieeex1_dynamics(x, ports, meta)` | `ieeex1_dynamics_jit(x, ports, meta)` | 5 | 15 |
| TGOV1     | `tgov1_dynamics(x, ports, meta)` | `tgov1_dynamics_jit(x, ports, meta)` | 2 | 10 |
| IEEEG1    | `ieeeg1_dynamics(x, ports, meta)` | `ieeeg1_dynamics_jit(x, ports, meta)` | 6 | 22 |

**Key design choices:**
- Each component defines index constants at module level (e.g., `_GM_M=0, _GM_D=1, ...`)
- `pack_<model>_meta(meta_dict)` functions convert Python dicts to flat `float64` arrays (called once at init)
- Pre-allocated port buffers eliminate per-call array allocation in the hot loop
- Original dict-based functions kept as fallback for non-JIT path
- Saturation coefficients (EXDC2, IEEEX1) pre-computed in `pack_meta()` for JIT efficiency
- ESST3A complex arithmetic uses explicit `cos/sin` instead of `np.exp(1j*...)` for Numba compatibility

**JIT Infrastructure** (`utils/numba_kernels.py`):
- `JIT_REGISTRY`: Maps model names to `(dynamics_jit, output_jit, pack_meta, ports_size, meta_size)`
- `prepare_jit_data(builder)`: Packs all component metadata into arrays, resolves JIT functions per machine
- `warmup_jit(jit_data, n_gen)`: Triggers compilation with dummy data before simulation starts

### LU Factorization Caching

Profiling revealed the true bottleneck was `np.linalg.solve` in the network solver (72% of runtime, ~125,000 calls), not component dynamics (3.5%). Since the Y_aug admittance matrix is constant between fault transitions:

- **Before**: `np.linalg.solve(Y_aug, I_total)` every iteration (O(n^3) factorization + O(n^2) solve)
- **After**: `lu_factor(Y_aug)` once per fault transition, `lu_solve(lu, I_total)` per iteration (O(n^2) solve only)
- Cache key: `(use_fault, fault_bus, fault_impedance)` — invalidated only on topology changes
- Both `Y_aug` and `Y_uu` (Kron-reduced) matrices are cached

### Performance Results

| System | Before | After | Speedup |
|--------|--------|-------|---------|
| Kundur 4-machine (t=15s) | 53s | 9s | **5.9x** |
| IEEE 14-bus 5-machine (t=15s) | 33s | 5s | **6.6x** |

**Usage**: JIT is enabled by default. Pass `use_jit=False` to `simulate()` to disable:
```python
sol = sim.simulate(x0, t_end=15.0)              # JIT enabled (default)
sol = sim.simulate(x0, t_end=15.0, use_jit=False)  # Fallback to dict-based
```

## Technical Details

### State Vector Organization
For an N-machine system with variable component models:
- Each generator (GENROU): 7 states (delta, omega, psi_d, psi_q, psi_f, psi_kd, psi_kq)
- Exciter states vary by model: EXDC2 (4), EXST1 (4), ESST3A (5)
- Governor states vary by model: TGOV1 (2), IEEEG1 (6)
- Example: IEEE 14-bus with 5 generators = 85 total states
- Dynamic sizing: System adapts to any number of generators and component types defined in JSON

### Network Solution

**Ybus Construction:**
- Admittance matrix built directly from Line data in JSON
- Per-unit values on common system base (e.g., 100 MVA)
- Standard π-model for lines and transformers
- Transformers: impedance given on common base, no turns ratio scaling needed

**Load Modeling:**
- **Constant power load model** with iterative solution
- Load current: `I_load = conj(S_load / V)` updated each iteration
- Converges in 3-5 iterations typically

**Generator-Network Interface:**
- Voltage-behind-reactance model: `E'' = V + jXd'' × I`
- Park transformation converts machine dq frame to system RI frame
- Generator admittance: `y_gen = 1/(j×Xd'')`

### Equilibrium Initialization

The framework uses a sophisticated 4-part initialization process that achieves **machine-precision equilibrium** (Max |dx/dt| < 1e-12):

**Part 1: AC Power Flow** (REQUIRED for perfect initialization)
- Newton-Raphson solver computes voltage magnitudes and angles at all buses
- Ensures consistent power balance across the network
- Provides accurate initial conditions for all electrical quantities
- **Enable via simulation JSON**: `"initialization": {"run_power_flow": true}`

**Part 2: Generator Internal States from Power Flow**
```python
# For each generator, from power flow results:
V_phasor = V_mag × exp(j × V_angle)    # Bus voltage
S = P_gen + j × Q_gen                   # Generator power
I_phasor = conj(S / V_phasor)          # Generator current
E_phasor = V_phasor + (ra + j×Xd'') × I_phasor  # Internal EMF
delta = angle(E_phasor)                 # Rotor angle
psi_f = |E_phasor| / gd1               # Field flux
Pm = Pe (from network solution)        # Mechanical power
```

**Part 3: Component Initialization (Exciters & Governors)**
- **Exciters**: Full signal path tracing from terminal voltage to field voltage
  - EXDC2: vm → vr → efd → xf, working backward to compute Vref
  - EXST1: vm → vll → vr → vf, backward chain to compute Vref (updated in metadata)
  - ESST3A: Limit-aware initialization matching all internal clamps:
    VE compensation → FEX(IN) → VB (VBMAX) → VM (VMMAX) → VG (VGMAX) → VR (VRMAX/VRMIN) → LL_exc_x (VIMAX/VIMIN) → Vref
- **Governors**: Initialize gate position and reference power from Pm
  - TGOV1: 2-state initialization (valve + turbine lag)
  - IEEEG1: 6-state initialization (lead-lag + valve + 4 turbine stages)
- All parameters read from component metadata (IEEE standards + JSON values)

**Part 4: Network Consistency Refinement**
- **Stator flux adjustment**: Recompute psi_d, psi_q from Park transformation
- **Exciter voltage sensing**: Update vm state with refined terminal voltage
- **Governor gate limits**: Enforce VMIN ≤ valve_position ≤ VMAX
  - **Critical**: VMIN must be ≤ actual operating point or system will drift
  - Example: If Pe = 0.217 pu, VMIN must be ≤ 0.217 (not 0.3!)

**Result**: Max |dx/dt| = 4.2e-13 (machine precision), ZERO angle drift

This approach handles complex exciter models (ESST3A with lead-lag blocks) that cannot be initialized algebraically. The key to perfect equilibrium is running power flow FIRST, then building component states consistently.

### Power Flow and Slack Bus Selection

**Automatic Slack Bus Selection:**

Every power system requires a slack bus to balance generation and load. The framework implements intelligent three-tier automatic selection:

1. **Priority 1 - User-Specified Slack** (highest priority)
   - Uses "Slack" section from JSON if present
   - Prefers grids/infinite buses over generators when multiple slacks exist
   
2. **Priority 2 - Grid/Voltage Source** (fallback)
   - Detects grid components (infinite bus models)
   - Ideal for SMIB (Single-Machine Infinite-Bus) systems
   
3. **Priority 3 - Largest Generator** (last resort)
   - Selects generator with highest MVA rating
   - Ensures multi-machine systems without explicit slack can converge

**Power Flow Features:**
- Newton-Raphson AC power flow solver
- Adaptive damping for robust convergence (even heavily loaded systems)
- Practical convergence tolerance (1e-4 pu)
- Up to 100 iterations for difficult cases
- Comprehensive diagnostics and validation
- Automatically updates system initial conditions

**Usage (REQUIRED for machine-precision initialization):**
```python
from utils.power_flow import run_power_flow

# Method 1: Run power flow explicitly before simulation
pf = run_power_flow(builder, coordinator, verbose=True)

# Method 2: Enable automatic power flow in simulation JSON (RECOMMENDED):
{
  "initialization": {
    "run_power_flow": true,
    "power_flow_verbose": true
  }
}
```

**Benefits:**
- **Machine-precision equilibrium**: Max |dx/dt| < 1e-12, ZERO drift
- Accurate voltage magnitudes and angles across all buses
- Consistent power injections and flows
- Proper slack bus power balancing
- No manual initial condition calculation required

**⚠️ Warning:** Skipping power flow results in poor initialization (drift, large transients). Always enable `run_power_flow: true` in simulation JSON for production simulations.

## Tips and Best Practices

### Lyapunov Stability Analysis

**Equilibrium Quality (with proper initialization):**
- **With power flow enabled** (`run_power_flow: true`): Max |dx/dt| < 1e-10 (machine precision)
- **Target**: All state derivatives should be effectively zero at t=0
- Rotor angle drift over simulation should be **< 0.01°** (ideally 0.00°)
- COI frame removes global rotation drift in multi-machine systems

**Common Initialization Issues:**
1. **Large Max |dx/dt| (> 0.1)**: Power flow not enabled → add `"run_power_flow": true` to simulation JSON
2. **Angle drift**: Governor limits preventing equilibrium → check VMIN ≤ actual operating point
3. **Exciter oscillations**: Case sensitivity in metadata → verify 'vref' not 'Vref'
4. **Gradual drift**: Pm ≠ Pe → power flow solver will fix this automatically

**Region of Attraction Estimation:**
- Start with 500 samples for initial assessment
- Increase to 1000-2000 for more accurate V_critical estimate
- max_perturbation = 0.5-1.0 rad for angle perturbations
- Stable fraction ~40-60% indicates reasonable stability margin

**Interpreting Results:**
- Unstable modes = 0: Linearized stable
- Marginal modes ≈ 16 (4-machine system): Expected (stator flux at ±377j, angle reference)
- V(t) decreasing: System dissipative and converging
- Energy margin > V_critical: Fault will cause loss of synchronism

### Impedance Scanning

**Frequency Range Selection:**
- Governor dynamics: 0.1 - 10 Hz
- AVR/Exciter dynamics: 5 - 50 Hz
- Network/Fast dynamics: 10 - 200 Hz
- Start with f_max = 50 Hz for balanced coverage

**Amplitude Selection:**
- Linear analysis: Use frequency-domain method
- Controller bandwidth: 0.01 - 0.05 pu (IMTB)
- Saturation study: 0.05 - 0.10 pu (Time-domain)
- If impedance changes with amplitude → system is nonlinear

**Interpreting Results:**
- **Resonances**: Sharp peaks in magnitude plot
- **Controller bandwidth**: -3dB point or phase crossover
- **Saturation**: Impedance decreases with amplitude
- **Discrepancies between methods**: Indicates nonlinearity
- **High impedance at low frequencies**: Normal for voltage-controlled generators

### General Tips

**System Building:**
- Start with example (Kundur system) and modify
- Verify power flow convergence before dynamics
- Check per-unit base consistency
- Use realistic parameter values

**Debugging:**
- Enable verbose output in system_builder
- Plot equilibrium states before simulation
- Check Jacobian condition number for numerical issues
- Compare with analytical results when available

## Troubleshooting Initialization Issues

### Problem: Large Max |dx/dt| at t=0 (> 0.1)

**Symptom**: Initial state derivatives are large, system drifts for several seconds before settling

**Solution**: Enable power flow in simulation JSON:
```json
{
  "initialization": {
    "run_power_flow": true,
    "power_flow_verbose": true
  }
}
```

**Why**: Power flow computes consistent voltage angles and magnitudes across all buses, ensuring all generators see correct terminal voltages from the start.

---

### Problem: Rotor Angle Drift (system accelerates/decelerates continuously)

**Symptom**: Rotor angle increases or decreases linearly, omega ≠ 1.0 in steady-state

**Root Cause**: Pm ≠ Pe (mechanical power doesn't match electrical power)

**Solution**: Power flow automatically adjusts Pm to match Pe. Verify:
1. `run_power_flow: true` in simulation JSON
2. Governor VMIN parameter ≤ actual operating point
3. Governor VMAX parameter ≥ actual operating point

**Example Issue**:
```json
// BAD: Operating point Pe = 0.217 pu, but VMIN = 0.3 forces Pm = 0.3
"TGOV1": {"VMIN": 0.3, "VMAX": 1.0}  // WRONG!

// GOOD: VMIN allows governor to reach actual operating point
"TGOV1": {"VMIN": 0.0, "VMAX": 1.0}  // CORRECT
```

---

### Problem: Exciter Voltage Oscillations at t=0

**Symptom**: Field voltage Efd oscillates or drifts, exciter states have large derivatives

**Possible Causes**:
1. **Case sensitivity**: Metadata key mismatch ('Vref' vs 'vref')
2. **Missing Vref initialization**: Some exciters require Vref in metadata
3. **Saturation function mismatch**: SE(Efd) calculation incorrect

**Solution**:
- Check component JSON for exact case of parameter names
- Verify exciter has 'vref'/'Vref' in metadata after initialization (set by init_fn)
- For EXDC2: saturation parameters SE_E1, SE_E2, E1, E2 must be correct
- For ESST3A: ensure VIMAX/VRMAX/Efd_max are large enough for the operating point (check init_fn output)
- For EXST1: Vref is automatically computed by init_fn from Vt and Efd_desired

---

### Problem: Power Flow Fails to Converge

**Symptom**: "Power flow did not converge" error, or unrealistic voltage magnitudes

**Solutions**:
1. **Check Slack Bus**: Ensure system has exactly one slack bus defined
2. **Check Load/Generation Balance**: ΣP_gen ≥ ΣP_load (slack will balance)
3. **Check Line Impedances**: Very low R or X can cause numerical issues
4. **Increase Iterations**: Power flow allows up to 100 iterations
5. **Check Per-Unit Base**: All quantities must be on same system base (e.g., 100 MVA)

---

### Problem: System Unstable Even With Perfect Initialization

**Symptom**: Max |dx/dt| = 0 at t=0, but system diverges after small disturbance

**This is NOT an initialization issue** - the system is genuinely unstable!

**Analysis Tools**:
1. **Lyapunov Eigenvalues**: Check for positive real parts
2. **Region of Attraction**: System may be locally stable but with small stability region
3. **Parameter Tuning**: Adjust exciter gains (KA), governor droop (R), damping (D)

Use `test_lyapunov.py` to analyze stability and identify problematic modes.

---

### Validation Checklist

After initialization, verify:
- [ ] Max |dx/dt| < 1e-10 (machine precision)
- [ ] Rotor angle drift < 0.01° over 10+ seconds
- [ ] Omega = 1.0 ± 1e-6 for all generators
- [ ] Power balance: ΣP_gen = ΣP_load (within 1e-4 pu)
- [ ] Voltage magnitudes: 0.95 < V < 1.10 pu at all buses

If all checks pass → **perfect initialization achieved!**

## Future Extensions

The modular architecture enables easy addition of:
- **Load models**: Static, dynamic, motor loads
- **FACTS devices**: SVC, STATCOM, UPFC
- **Renewable integration**: Wind turbines, solar PV with inverters
- **Protection systems**: Relay models, breaker logic
- **Advanced analysis**: 
  - ✅ Eigenvalue analysis (Lyapunov analyzer)
  - ✅ Lyapunov stability certificates
  - Participation factors and mode shapes
  - Sensitivity analysis
- **Optimization tools**: Optimal power flow, control tuning
- **Real-time simulation**: Hardware-in-the-loop capabilities
- **Stability enhancements**:
  - Hybrid energy-Lyapunov functions
  - Constructive Lyapunov design for control
  - Critical clearing time (CCT) calculation

## Contributing

To contribute new models or analysis tools:
1. Follow the Port-Hamiltonian structure conventions
2. Use DynamicsCore pattern for component models
3. Add comprehensive documentation
4. Create test cases demonstrating functionality
5. Update README and example documentation

## References

- Port-Hamiltonian modeling of electrical networks
- Kundur, P. (1994). *Power System Stability and Control*. McGraw-Hill.
- IEEE standard models for generators, exciters, and governors
- van der Schaft, A. J. (2000). *L2-Gain and Passivity Techniques in Nonlinear Control*. Springer.

---

**Framework Version**: 1.6
**Last Updated**: February 2026

### Changelog

**v1.6 (February 2026)**
- **NEW: Numba JIT Compilation for Component Dynamics (~6x overall speedup)**
  - All 7 component models (GENROU, EXDC2, EXST1, ESST3A, IEEEX1, TGOV1, IEEEG1) have `@njit(cache=True)` compiled versions
  - Array-based function signatures replace Python dicts for Numba compatibility
  - Pre-allocated port buffers eliminate per-call allocation in the ODE hot loop
  - `utils/numba_kernels.py`: JIT registry, `prepare_jit_data()`, `warmup_jit()` infrastructure
  - ESST3A complex arithmetic converted to explicit real/imag `cos/sin` operations
  - EXDC2/IEEEX1 saturation coefficients pre-computed in `pack_meta()` for JIT efficiency
  - Each component has `pack_<model>_meta()`, `<model>_dynamics_jit()`, `<model>_output_jit()`
- **NEW: LU Factorization Caching in Network Solver**
  - `system_coordinator.py`: Y_aug factored once per fault transition using `scipy.linalg.lu_factor`
  - `lu_solve` per iteration instead of full `np.linalg.solve` (O(n^2) vs O(n^3))
  - Cache key based on fault state: `(use_fault, fault_bus, fault_impedance)`
  - Both Y_aug and Y_uu (Kron-reduced) matrices cached
  - Profiling identified `np.linalg.solve` as 72% of runtime (not component dynamics at 3.5%)
- **Performance Results**: Kundur 53s→9s (5.9x), IEEE 14-bus 33s→5s (6.6x)
- **Bug Fix**: `KeyError: 'R'` in power_flow.py for IEEEG1 governor (uses 'K' gain, not 'R' droop)
  - Added guard: `if 'R' in builder.gov_metadata[i]:` before accessing droop parameter

**v1.5 (February 2026)**
- **NEW: IEEE 14-Bus and IEEE 39-Bus Test Systems**
  - IEEE 14-bus: 5 generators with mixed exciter/governor models (ESST3A/EXST1 + IEEEG1/TGOV1)
  - IEEE 39-bus (New England): 10 generators with ESST3A + IEEEG1
  - Both achieve machine-precision equilibrium (Max |dx/dt| < 1e-12)
- **NEW: EXST1 Static Exciter (IEEE ST1)**
  - 4-state model: voltage measurement, lead-lag compensator, regulator, washout feedback
  - Dynamic output limits with field current compensation (KC * XadIfd)
  - Anti-windup on regulator limits (VRMAX/VRMIN)
  - Proper Vref back-calculation during initialization
- **NEW: ESST3A Static Exciter (IEEE ST3A)**
  - 5-state model with inner/outer voltage regulators
  - VE compensation and FEX rectifier function (IN-based)
  - Lead-lag compensator with input voltage limiters (VIMAX/VIMIN)
  - Comprehensive limit-aware initialization matching ALL internal clamps:
    VBMAX, VMMAX, VGMAX, VRMAX/VRMIN, VIMAX/VIMIN, Efd_max
  - Handles operating points where multiple limiters are simultaneously active
- **NEW: IEEEG1 Multi-Stage Steam Turbine Governor (IEEE G1)**
  - 6-state model: lead-lag filter, valve position, 4 turbine/reheater stages
  - Rate-limited valve speed (UO/UC) with anti-windup (PMIN/PMAX)
  - Power fraction distribution across stages (K1-K8, auto-normalized)
  - HP + LP turbine output computation
- **Critical Bug Fixes for Exciter Initialization**
  - Fixed Vref overwrite bug in power_flow.py: init_fn computed correct Vref but power_flow.py was overwriting it with Vt_actual, destroying the voltage error offset needed for equilibrium
  - Fixed EXST1 init_fn: was discarding computed Vref (lambda returned x0 only); now properly updates metadata['Vref']
  - Fixed ESST3A initialization to match ALL internal limits in dynamics function (VGMAX, VMMAX, VBMAX, hard Efd clip), preventing derivative mismatch at t=0
  - Removed overly restrictive vref clip [0.8, 1.2] in ESST3A dynamics that prevented high-Vref operating points
  - Fixed ESST3A_2 parameters in IEEE 14-bus JSON (VIMAX, VRMAX, Efd_max were too restrictive for operating point)

**v1.4 (February 2026)**
- **NEW: 4-Part Equilibrium Initialization Process**
  - Part 1: AC power flow for consistent voltage angles/magnitudes
  - Part 2: Generator internal states from power flow results
  - Part 3: Component initialization (full signal path tracing for exciters/governors)
  - Part 4: Network consistency refinement (stator flux, voltage sensing, gate limits)
  - **Achieves machine-precision equilibrium**: Max |dx/dt| < 1e-12, ZERO drift
  - Handles complex exciter models (ESST3A, EXDC2) with IEEE-standard initialization
  - Automatic governor limit enforcement (VMIN/VMAX)
- **NEW: AC Power Flow with Automatic Slack Bus Selection**
  - Three-tier intelligent slack bus selection (user → grid → largest generator)
  - Newton-Raphson solver with adaptive damping
  - Robust convergence for heavily loaded systems (up to 100 iterations)
  - Practical convergence criteria (1e-4 pu)
  - Comprehensive power balance validation
  - **Required for perfect initialization** - enable via `"run_power_flow": true` in simulation JSON
- **Modular Architecture Enhancements**
  - Component self-description (`n_states`, `output_fn`, `component_type`, `model_name`)
  - Dynamic component registry with flat hierarchical structure
  - Zero hardcoded component logic in simulator
  - Full support for component swapping via JSON only
- **Two-File JSON Configuration**
  - Separate system parameters (system.json) and simulation settings (simulation.json)
  - Improved reusability and configuration management
- **Bug Fixes**
  - Fixed GENROU stator flux sign error (ra term)
  - Fixed TGOV1 spurious 10x factors
  - Rewrote EXDC2 to full IEEE 4-state model
  - Fixed exciter metadata case sensitivity ('vref' vs 'Vref')

**v1.3 (January 2026)**
- **NEW: Lyapunov Stability Analyzer**
  - Energy-based stability analysis leveraging Port-Hamiltonian structure
  - Linearized stability via Jacobian eigenvalue decomposition
  - Proper Lyapunov function formulation: V(x*) = 0, V(x) > 0
  - Center-of-Inertia (COI) frame for rotational invariance
  - Region of attraction estimation (10-20x faster using dV/dt criterion)
  - Transient energy margin computation
  - Passivity verification through energy evolution
  - Comprehensive multi-panel visualization
- **DynamicsCore Pattern**
  - Extended Core class with numerical dynamics capability
  - Enables general Lyapunov analysis across arbitrary component combinations
  - Each component provides its own dynamics function
- **Documentation Split**
  - Framework documentation in README.md
  - Example-specific details in test_cases/Kundur_System/EXAMPLE.md

**v1.2 (January 2026)**
- **Time-Domain Impedance Scanner Major Fix**
  - Implemented trajectory difference method for dynamic impedance measurement
  - Runs baseline + injection simulations
  - Fixed flat impedance issue (now captures dynamic flux response)
- **Exciter Equilibrium Fix (all modules)**
  - Fixed Vref initialization: Vref = Vt + Efd/KA
  - Prevents Efd drift during long simulations
- **ASCII Progress Bar**
  - Windows compatibility improvements

**v1.1 (January 2026)**
- Fixed Ybus transformer scaling bug
- Fixed double per-unit base conversion
- Implemented constant power load model
- Rewrote equilibrium initialization using direct power flow
- Made fault_sim_modular fully dynamic (any number of generators)

**v1.0 (January 2026)**
- Initial release with modular Port-Hamiltonian framework
- Three impedance scanning methods
- Fault simulation
- JSON-based system configuration

---

## Renewable Energy Integration - Current Status (Feb 2026)

### ✅ Completed Implementation

The framework now includes a full Type-3 Wind Turbine (WT3) model with 7 sub-components:
- **REGCA1** - Renewable Generator/Converter (current source inverter)
- **REECA1** - Electrical Control (current command generation)
- **REPCA1** - Plant Controller (plant-level P/Q control)
- **WTDTA1** - Drive Train (two-mass mechanical model)
- **WTARA1** - Aerodynamics (power vs pitch)
- **WTPTA1** - Pitch Control
- **WTTQA1** - Torque Control

All models are implemented as Port-Hamiltonian systems in `components/renewables/`.

**Key Infrastructure Updates:**
- `component_factory.py` - Registers renewable models
- `system_builder.py` - Builds and links WT3 sub-components  
- `system_coordinator.py` - Handles converter current injection
- `fault_sim_modular.py` - Integrates renewable dynamics
- `power_flow.py` - Initializes renewable states

**Test System:**
- IEEE 14-bus with WT3 at bus 8 (replacing GENROU_5)
- Configuration: `test_cases/ieee14bus/renewable_resources_adoption/ieee14_wt3_system.json`

### ⚠️ Known Issue: Initialization Convergence

**Problem:** The WT3 system has large initial derivatives (~3.7) in REPCA1 and REECA1 PI controller states, indicating the system is not at equilibrium.

**Root Cause:** PI controllers in REPCA1/REECA1 require **integrator pre-loading** for equilibrium initialization. Currently, all PI integrators are initialized to zero, which assumes zero error. However, for a PI controller to produce a non-zero output with zero error, the integrator must be pre-loaded:

```
PI output = Kp * error + Ki * integrator
At equilibrium (error=0): output_eq = Ki * integrator_eq
Therefore: integrator_eq = output_eq / Ki
```

**Testing Evidence:**
- Base IEEE 14-bus (no renewables): `Max |dx/dt| = 2.9e-13` ✅ Perfect
- With WT3 (full controls): `Max |dx/dt| = 3.7` ❌ Poor
- Largest derivatives in: REPCA1 Q-filter (-3.7), REECA1 V-filter (-3.6)

### 🔧 Recommended Fixes

**Option 1: Sophisticated Initialization (Recommended for production)**

Update `init_fn` for REPCA1/REECA1 to:
1. Calculate required outputs (Pext, Qext, Ipcmd, Iqcmd) from power flow
2. Back-calculate PI integrator states to produce those outputs with zero error
3. Handle cascaded control dependencies (REPCA1 → REECA1 → REGCA1)

**Option 2: Reduced Control Authority (Quick fix for testing)**
- Lower PI gains (Kp, Ki) in JSON to reduce initial transients
- Use longer filter time constants (Trv, Tp, Tfltr)
- This trades off control performance for stability

**Option 3: Gradual Enablement (Practical approach)**
- Start simulation with fixed current commands from power flow
- Gradually ramp up control authority over first few seconds
- Allows system to settle before full dynamic control engages

**Option 4: Simplified Model (Development/testing)**
- Test REGCA1 alone with fixed Ipcmd/Iqcmd from power flow
- Add control layers incrementally: REECA1 → REPCA1 → Mechanical
- Validate each layer separately before full integration

### 📁 Related Files
- Implementation: `components/renewables/*.py`
- Test scripts: `test_renewable_initialization.py`, `test_renewable_simulation.py`
- Configuration: `test_cases/ieee14bus/renewable_resources_adoption/`
