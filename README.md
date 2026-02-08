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
│   ├── fault_sim_modular.py     # ⭐ Fault simulation module
│   ├── lyapunov_analyzer.py     # ⭐ Lyapunov stability analysis
│   ├── impedance_scanner.py     # ⭐ Frequency-domain impedance
│   ├── imtb_scanner.py          # ⭐ IMTB multisine impedance scanner
│   ├── impedance_scanner_td.py  # ⭐ Time-domain white noise impedance scanner
│   ├── im_analysis_lib.py       # Impedance analysis utilities and plotting
│   └── model_templates.py       # Component templates for rapid development
│
├── test_cases/                   # System Configuration Files (JSON)
│   ├── Kundur_System/           # Example: Kundur's 4-machine system
│   │   ├── kundur_full.json     # Complete system definition
│   │   ├── system_config.json   # Alternative configuration
│   │   └── EXAMPLE.md           # 📖 Detailed example documentation
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

# Initialize equilibrium (automatically runs numerical refinement if needed)
x0 = sim.initialize_equilibrium()

# Run simulation
sol = sim.simulate(x0, t_end=15.0)
sim.plot_results(sol)
```

**Simulation JSON Example:**
```json
{
  "time": {"t_end": 15.0, "n_points": 3000},
  "fault": {"enabled": true, "bus": 1, "impedance": "0.01j", 
            "start_time": 2.0, "duration": 0.15},
  "solver": {"method": "Radau", "rtol": 1e-6, "atol": 1e-8},
  "initialization": {"run_power_flow": true}
}
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
python test_modular_fault.py        # Test fault simulation
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

3. **Run with your configuration**:
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
- [Expandable]: Add SEXS, IEEET1, custom exciters

**governors/** - Governor Models  
- `tgov1.py`: Steam turbine governor type 1
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

## Technical Details

### State Vector Organization
For an N-machine system:
- Each generator: 13 states (7 electrical + 4 exciter + 2 governor)
- Total: 13×N states for complete system dynamics
- Dynamic sizing: System adapts to any number of generators defined in JSON

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

The framework uses a sophisticated multi-stage initialization process to ensure zero drift:

**Stage 1: Algebraic Equilibrium Guess**
```python
# For each generator:
V_phasor = V_mag × exp(j × V_angle)    # From Bus data
S = P_gen + j × Q_gen                   # From PV/Slack data
I_phasor = conj(S / V_phasor)          # Generator current
E_phasor = V_phasor + (ra + j×Xd'') × I_phasor  # Internal EMF
delta = angle(E_phasor)                 # Rotor angle
psi_f = |E_phasor| / gd1               # Field flux
```

**Stage 2: Mechanical Power Adjustment**
- Computes actual electrical power from network solver
- Adjusts governor mechanical power Pm to match Pe exactly
- Ensures zero rotor acceleration: `d(omega)/dt = (Pm - Pe) / M = 0`

**Stage 3: Adaptive Numerical Refinement**
- Automatically detects large derivatives (exciter/governor transients)
- Simulates system until all derivatives converge
- Checks convergence adaptively (no hardcoded settling time)
- Typically converges in 5-20 seconds of simulation
- Achieves near-zero drift (< 0.1° over 15s)

This approach handles complex exciter models (like ESST3A with lead-lag blocks) that cannot be initialized algebraically.

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

**Usage:**
```python
from utils.power_flow import run_power_flow

# Optional - run power flow before simulation
pf = run_power_flow(builder, coordinator, verbose=True)

# Or enable automatic power flow in simulation JSON:
{
  "initialization": {
    "run_power_flow": true,
    "power_flow_verbose": true
  }
}
```

**Benefits:**
- Accurate voltage magnitudes and angles across all buses
- Consistent power injections and flows
- Proper slack bus power balancing
- Reduced initialization transients
- No manual initial condition calculation required

## Tips and Best Practices

### Lyapunov Stability Analysis

**Equilibrium Quality:**
- Max |dx/dt| ~0.2 from fast stator flux dynamics is acceptable
- Slow dynamics (angle, speed, field) should have |dx/dt| < 0.01
- COI frame removes global rotation drift
- If equilibrium drifts, check Vref and Pref initialization

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

**Framework Version**: 1.4
**Last Updated**: February 2026

### Changelog

**v1.4 (February 2026)**
- **NEW: Adaptive Numerical Equilibrium Solver**
  - Multi-stage initialization: algebraic guess → Pm adjustment → adaptive refinement
  - Automatically detects and corrects large exciter/governor derivatives
  - Adaptive convergence monitoring (no hardcoded settling time)
  - Achieves near-perfect equilibrium (< 0.1° drift over 15s)
  - Handles complex exciter models (ESST3A, etc.) that cannot be initialized algebraically
- **NEW: AC Power Flow with Automatic Slack Bus Selection**
  - Three-tier intelligent slack bus selection (user → grid → largest generator)
  - Newton-Raphson solver with adaptive damping
  - Robust convergence for heavily loaded systems (up to 100 iterations)
  - Practical convergence criteria (1e-4 pu)
  - Comprehensive power balance validation
  - Optional automatic execution via simulation JSON
- **Modular Architecture Enhancements**
  - Component self-description (`n_states`, `output_fn`, `component_type`, `model_name`)
  - Dynamic component registry with flat hierarchical structure
  - Zero hardcoded component logic in simulator
  - Full support for component swapping via JSON only
- **Two-File JSON Configuration**
  - Separate system parameters (system.json) and simulation settings (simulation.json)
  - Improved reusability and configuration management

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
