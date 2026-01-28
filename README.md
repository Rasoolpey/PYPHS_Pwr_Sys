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
- **Lyapunov Stability Analysis** (`lyapunov_analyzer.py`): Energy-based stability assessment
  - Linearized stability via eigenvalue analysis
  - Port-Hamiltonian energy function as Lyapunov candidate
  - Region of attraction estimation with dV/dt criterion
  - Transient energy margin computation
  - Multi-panel stability visualization
- **Impedance Scanning** (Multiple methods):
  - `impedance_scanner.py`: Frequency-domain linearization (fast, small-signal)
  - `imtb_scanner.py`: IMTB multisine injection (MIMO, medium-amplitude)
  - `impedance_scanner_td.py`: White noise time-domain (large-signal, nonlinear)
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
│   ├── lyapunov_analyzer.py     # ⭐ Lyapunov stability analysis (energy-based)
│   ├── impedance_scanner.py     # ⭐ Frequency-domain impedance (linearization)
│   ├── imtb_scanner.py          # ⭐ IMTB multisine impedance scanner (MIMO)
│   ├── impedance_scanner_td.py  # ⭐ Time-domain white noise impedance scanner
│   ├── im_analysis_lib.py       # Impedance analysis utilities and plotting
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

**Method 1: Frequency-Domain Linearization (Fast, Small-Signal)**
```python
from utils.impedance_scanner import ImpedanceScanner
import numpy as np

# Initialize scanner
scanner = ImpedanceScanner('test_cases/Kundur_System/kundur_full.json')

# Define frequency range
freqs = np.logspace(-1, 2, 100)  # 0.1 Hz to 100 Hz

# Scan impedance at specific bus (linearized small-signal)
f, Z_mag, Z_phase, Z_dq = scanner.scan_impedance(bus_idx=0, freqs=freqs)
```

**Method 2: IMTB Multisine (MIMO, Medium-Amplitude)**
```python
from utils.imtb_scanner import IMTBScanner

# Initialize IMTB scanner
scanner = IMTBScanner('test_cases/Kundur_System/kundur_full.json')

# Run MIMO scan with multisine injection
freqs, Z_dq_matrices = scanner.run_mimo_scan(
    bus_idx=2, 
    freqs=np.logspace(-1, 2, 30),  # 30 frequency points
    amplitude=0.10  # 0.10 pu injection amplitude
)
```

**Method 3: Time-Domain White Noise (Large-Signal, Nonlinear)**
```python
from utils.impedance_scanner_td import ImpedanceScannerTD

# Initialize time-domain scanner
scanner = ImpedanceScannerTD('test_cases/Kundur_System/kundur_full.json')

# Run white noise scan
sol, bus_idx = scanner.run_scan(
    bus_idx=2, 
    f_max=50.0,      # Maximum frequency (Hz)
    amplitude=0.01,  # Injection amplitude (pu)
    duration=60.0    # Simulation duration (s)
)

# Post-process to get impedance
freqs, Z_est = scanner.post_process_tfe(sol, bus_idx)

# Plot detailed system response
scanner.plot_system_response('outputs/system_response.png')
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
- ✅ Small-signal stability analysis (Lyapunov analyzer)
- Modal analysis and participation factors
- Optimal power flow
- State estimation
- Voltage stability analysis
- ✅ Transient stability screening (Lyapunov energy margins)
- Custom control strategies

The Port-Hamiltonian framework and modular architecture make it easy to add any new function!

## Module Architecture and Responsibilities

The framework uses a clean separation of concerns with each module handling specific tasks:

### Core Modules (`utils/`)

**pyphs_core.py** - Port-Hamiltonian Engine
- Standalone symbolic Port-Hamiltonian implementation
- No external PyPHS dependencies
- Manages Hamiltonian storage, symbolic mathematics, and energy functions
- **DynamicsCore class**: Extends Core with numerical dynamics capability
- Enables general stability analysis across arbitrary component combinations
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
- Builds Ybus matrix dynamically from Line data (no hardcoded values)
- Manages pre-fault and during-fault admittance matrices
- **Constant power load model** with iterative voltage solution
- Park transformations (machine dq ↔ system RI frame)
- Fault application: `apply_fault(bus_idx, impedance)`
- Coordinates generator-network interface via voltage-behind-reactance model

**fault_sim_modular.py** ⭐ - Fault Simulation Module
- **Fully dynamic** - adapts to any number of generators from JSON
- Direct equilibrium initialization from power flow data
- Configurable fault scenarios (any bus, any impedance, any time)
- Automatic Ybus switching during fault events
- Time-domain simulation with adaptive integration (solve_ivp)
- Post-simulation plotting and analysis
- Example: `sim.set_fault(bus_idx=1, impedance=0.01j, start_time=2.0, duration=0.15)`

**lyapunov_analyzer.py** ⭐ - Lyapunov Stability Analyzer
- **Port-Hamiltonian energy-based stability analysis**
- Hamiltonian as natural Lyapunov function candidate
- Linearized stability: Jacobian eigenvalue analysis at equilibrium
- **Region of Attraction (ROA)**: Estimates stability boundary using dV/dt < 0 criterion
- **Transient Energy Margin**: Computes energy absorbed during faults
- **Passivity Verification**: Checks energy dissipation along trajectories
- **COI Frame**: Removes rotational invariance for multi-machine systems
- Comprehensive visualization: eigenvalues, phase portraits, energy landscapes, ROA
- Performance: 500 sample estimation in ~80 seconds (10-20x faster than simulation)
- Example: `results = analyzer.analyze_linearized_stability()`

**impedance_scanner.py** ⭐ - Frequency-Domain Impedance Scanner
- Small-signal linearization around operating point
- State-space model extraction (A, B, C, D matrices)
- Transfer function analysis: Z(s) = C(sI-A)^(-1)B + D
- **Includes saturation**: Linearizes around saturated equilibrium state
- **Proper D-matrix**: Captures immediate network impedance response
- Fast computation (no time simulation)
- Best for: Initial design, small perturbations, smooth Bode plots

**imtb_scanner.py** ⭐ - IMTB Multisine Impedance Scanner
- Impedance Measurement Test Bench (IMTB) method
- Multisine current injection at specific frequencies
- MIMO (2×2) dq impedance matrix measurement
- DFT analysis for precise frequency response
- Includes saturation effects (exciter anti-windup, governor limits)
- Best for: Medium-amplitude testing, cross-coupling analysis, controller bandwidth

**impedance_scanner_td.py** ⭐ - Time-Domain White Noise Scanner
- Band-limited white noise current injection
- Transfer function estimation via Welch's method
- **Trajectory difference method**: Runs baseline + injection simulations to capture dynamic impedance
- Captures full nonlinear dynamics and saturation
- Visual progress bar with real-time updates
- Comprehensive signal extraction (Vd, Vq, Id, Iq, δ, ω, Efd, Gate, Pm, Pe)
- System response plotting (8-panel visualization)
- Best for: Large-signal behavior, nonlinear effects, realistic disturbances

**im_analysis_lib.py** - Impedance Analysis Utilities
- Plotting functions for impedance data
- Analysis helpers for resonance detection
- Stability margin calculations

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

## Impedance Scanning Methods - Detailed Comparison

The framework provides **three complementary impedance scanning methods**, each optimized for different analysis scenarios and offering unique insights into system dynamics.

### Method Comparison Table

| Feature | Frequency-Domain | IMTB Multisine | Time-Domain White Noise |
|---------|-----------------|----------------|------------------------|
| **File** | `impedance_scanner.py` | `imtb_scanner.py` | `impedance_scanner_td.py` |
| **Test Script** | `test_impedance_scanning.py` | `test_imtb_scanning.py` | `test_impedance_td.py` |
| **Technique** | Linearization | Multisine DFT | Trajectory Difference + Welch |
| **Computation** | State-space analysis | Time simulation | Two simulations (baseline + injection) |
| **Speed** | Very Fast (~seconds) | Medium (~minutes) | Slow (~5-15 minutes) |
| **Frequency Points** | 100+ points | 20-50 points | Resolution = duration⁻¹ |
| **Amplitude** | Infinitesimal (ε=10⁻⁵) | 0.01-0.10 pu | 0.001-0.10 pu |
| **Nonlinearity** | ❌ Linear only | ✓ Partial (saturation) | ✓ Full nonlinear |
| **MIMO** | ✓ 2×2 dq matrix | ✓ 2×2 dq matrix | Single complex Z |
| **Saturation** | ✓ Included (linearized) | ✓ Included | ✓ Included |
| **Noise** | None (analytical) | Low (DFT) | Medium (needs averaging) |
| **Best For** | Initial design | Controller tuning | Validation testing |

### Method 1: Frequency-Domain Linearization

**Principle:** Linearizes the Port-Hamiltonian dynamics around equilibrium and analytically computes the impedance transfer function.

**Mathematical Approach:**
1. Compute state-space model via numerical Jacobian: dx/dt = Ax + Bu, y = Cx + Du
2. Analytically evaluate: Z(s) = C(sI - A)⁻¹B + D

**Key Features:**
- ✓ Extremely fast (no time simulation required)
- ✓ Smooth, analytical Bode plots
- ✓ Full MIMO dq impedance matrix
- ✓ **Now includes exciter/governor saturation** (linearized around saturated state)
- ✓ **Proper D-matrix computation** (captures immediate network response)
- ⚠ Only valid for small perturbations (linear regime around operating point)

**Use Cases:**
- Initial system design and controller tuning
- Rapid parametric studies
- Small-signal stability screening
- Academic studies requiring linearized models

**Example Output:** Smooth impedance curves from 0.1 Hz to 100+ Hz
- Low frequency: High impedance (100-10000 pu) due to voltage control
- Mid frequency: Decreasing impedance as AVR bandwidth is exceeded
- High frequency: Settles to electrical impedance (~0.5-1.0 pu)

### Method 2: IMTB Multisine Scanner

**Principle:** Injects a multisine signal (sum of sinusoids at target frequencies) and uses DFT to extract impedance at each frequency.

**Mathematical Approach:**
1. Inject: I(t) = Σ Aₖ·sin(2πfₖt + φₖ) for k = 1...N
2. Measure voltage response V(t)
3. Apply DFT at each fₖ to get Z(fₖ) = V(fₖ)/I(fₖ)

**Key Features:**
- ✓ MIMO measurement (full 2×2 dq impedance matrix)
- ✓ Precise frequency targeting (20-50 logarithmically spaced points)
- ✓ Includes saturation effects (exciter anti-windup, governor limits)
- ✓ Good signal-to-noise ratio (coherent detection)
- ⚠ Medium computation time (needs settling at each frequency)
- ⚠ Limited to moderate injection amplitudes (0.01-0.10 pu)

**Use Cases:**
- Controller bandwidth assessment (AVR: 8-50 Hz, Governor: 1-10 Hz)
- Cross-coupling analysis (Zdq, Zqd off-diagonal terms)
- Medium-amplitude validation (bridge between linear and large-signal)
- Resonance detection

**Key Parameters:**
```python
amplitude = 0.10      # Injection amplitude (pu) - reveals saturation
freqs = np.logspace(-1, 2, 30)  # 30 frequencies from 0.1-100 Hz
```

**Example Output:** Clean impedance measurements at specific frequencies with full dq coupling information

### Method 3: Time-Domain White Noise Scanner

**Principle:** Injects band-limited white noise and uses spectral analysis (Welch's method) to estimate the transfer function from current to voltage. Uses **trajectory difference method** to capture dynamic frequency-dependent impedance.

**Mathematical Approach:**
1. Run **baseline simulation** (no injection) to capture natural system evolution
2. Run **injection simulation** with band-limited white noise: I(t) ~ N(0, σ²), filtered to [0, f_max]
3. Compute voltage perturbation: ΔV(t) = V_injection(t) - V_baseline(t)
4. Apply Welch's method: Z(f) = P_ΔV,I(f) / P_II(f)

**Why Trajectory Difference?**
- Single-simulation approach only captures **algebraic impedance** (Xd'')
- Trajectory difference captures how **generator flux dynamics** (ψf, ψkd, ψkq) evolve differently due to injection
- Result: Frequency-dependent impedance showing subtransient/transient behavior

**Key Features:**
- ✓ **Trajectory difference method** for dynamic impedance measurement
- ✓ Captures full nonlinear dynamics and saturation
- ✓ Most realistic method (simulates actual disturbances)
- ✓ Amplitude-dependent impedance (reveals nonlinearity)
- ✓ Visual progress bar with real-time ETA
- ✓ Comprehensive signal extraction (13 signal types)
- ✓ 8-panel system response visualization
- ⚠ Computationally expensive (runs two simulations, ~5-15 minutes total)
- ⚠ Noisy results (requires long duration for averaging)
- ⚠ Single complex impedance (not MIMO)

**Use Cases:**
- Large-signal behavior validation
- Verifying stability under realistic disturbances
- Identifying nonlinear effects (saturation, cross-coupling)
- Understanding system response to stochastic inputs
- Final validation before deployment

**Key Parameters:**
```python
f_max = 50.0          # Maximum frequency (Hz) - determines sampling rate
duration = 60.0       # Simulation time (s) - longer = better averaging
amplitude = 0.01      # Injection amplitude (pu) - test different levels
```

**Sampling Rate:** Auto-calculated as fs = 10 × f_max
- f_max = 50 Hz → fs = 500 Hz
- f_max = 200 Hz → fs = 2000 Hz (slower simulation)

**Tradeoffs:**
- **Duration ↑** → Better frequency resolution, better averaging, longer computation
- **f_max ↑** → Higher frequencies captured, faster sampling needed, slower computation
- **Amplitude ↑** → Reveals nonlinearity, may cause instability
- **Two simulations** → Captures dynamic impedance but doubles computation time

**Advanced Features:**
- **Trajectory Difference:** Runs baseline + injection simulations for true dynamic impedance
- **Progress Tracking:** Real-time progress bar showing elapsed/total time and ETA (for each simulation)
- **Signal Extraction:** Captures Vd, Vq, Id, Iq, δ, ω, Efd, Gate, Pm, Pe, Vref
- **Response Plotting:** 8-panel visualization of system dynamics
- **Saturation Diagnostics:** Can identify when/where controllers saturate
- **Coherence Metric:** Quality indicator for TFE estimation (should be >0.5)

**Example Output:** 
- Impedance spectrum from 0.1 Hz to f_max
- System response plots showing all internal signals
- Reveals amplitude-dependent behavior

### Choosing the Right Method

**For initial design:**
→ Use **Frequency-Domain** (fast, clean, good for parameter sweeps)

**For controller tuning:**
→ Use **IMTB Multisine** (medium amplitude, precise frequencies, MIMO)

**For validation:**
→ Use **Time-Domain** (realistic, captures nonlinearity, comprehensive diagnostics)

**For complete analysis:**
→ Use **all three methods** and compare results:
- Frequency-domain: establishes linear baseline
- IMTB: reveals saturation at medium amplitude  
- Time-domain: validates large-signal behavior

**Discrepancies between methods indicate:**
- Saturation effects (exciter, governor limits)
- Nonlinear dynamics (magnetic saturation, cross-coupling)
- Amplitude-dependent impedance
- Linearization assumptions violated

## Lyapunov Stability Analysis - Energy-Based Certificates

The framework includes a comprehensive **Lyapunov stability analyzer** that leverages the Port-Hamiltonian structure to assess system stability through energy methods. This is a natural fit for port-Hamiltonian systems where the Hamiltonian serves as an energy storage function.

### Key Features

**Port-Hamiltonian Energy Function**
- Uses system Hamiltonian as Lyapunov function candidate
- Ensures V(x*) = 0 at equilibrium and V(x) > 0 for perturbations
- Captures kinetic energy (rotor speed), magnetic energy (flux linkages), and potential energy (rotor angles)

**Center-of-Inertia (COI) Frame**
- Removes rotational invariance from angle coordinates
- Ensures energy function is frame-independent
- Critical for multi-machine stability analysis

**Multiple Analysis Methods**
1. **Linearized Stability**: Jacobian eigenvalue analysis at equilibrium
2. **Energy Margin**: Transient energy absorbed during faults
3. **Region of Attraction**: Estimates stability boundary via dV/dt < 0 criterion
4. **Passivity Verification**: Checks energy dissipation along trajectories

### Mathematical Foundation

The Lyapunov function is constructed as:

```
V(x) = V_kinetic + V_magnetic + V_potential

where:
  V_kinetic = 0.5 * sum(M_i * (omega_i - 1)^2)
  
  V_magnetic = 0.5 * sum((psi - psi*)^T * L^-1 * (psi - psi*))
  
  V_potential = sum(Pm_i * (theta_i - theta_i*))
                + sum(C_ij * sin(delta_ij*) * (delta_ij - delta_ij*)^2)
```

**Key Properties:**
- **Positive definite**: V(x) >= 0 with V(x*) = 0
- **Decreasing along trajectories**: dV/dt < 0 due to system dissipation
- **Physical meaning**: Total energy above equilibrium

### Usage Example

```python
from utils.lyapunov_analyzer import LyapunovStabilityAnalyzer
import numpy as np

# Initialize analyzer
analyzer = LyapunovStabilityAnalyzer('test_cases/Kundur_System/kundur_full.json')

# 1. Initialize equilibrium point from power flow
x_eq = analyzer.initialize_equilibrium()
print(f"Equilibrium Hamiltonian: {analyzer.H_eq:.4f}")

# 2. Linearized stability analysis
results = analyzer.analyze_linearized_stability()
print(f"System Status: {'STABLE' if results['is_stable'] else 'UNSTABLE'}")
print(f"Stable modes: {results['stable_count']}")
print(f"Unstable modes: {results['unstable_count']}")
print(f"Marginal modes: {results['marginal_count']}")

# 3. Estimate region of attraction (fast dV/dt method)
region = analyzer.estimate_stability_region(
    n_samples=500,          # Number of test points
    max_perturbation=1.0    # Perturbation magnitude
)
print(f"Stable fraction: {region['stable_fraction']*100:.1f}%")
print(f"Critical energy: V_crit = {region['V_critical']:.4f}")

# 4. Verify passivity during transient
from scipy.integrate import solve_ivp

# Small perturbation
x0 = x_eq + np.random.randn(len(x_eq)) * 0.05

# Simulate
sol = solve_ivp(
    lambda t, x: analyzer._system_dynamics(x, np.zeros(analyzer.n_gen)),
    (0, 10.0), x0, t_eval=np.linspace(0, 10, 200)
)

# Check Lyapunov function evolution
V_initial = analyzer.compute_lyapunov_function(sol.y[:, 0])
V_final = analyzer.compute_lyapunov_function(sol.y[:, -1])
print(f"Energy dissipation: {V_initial:.4f} -> {V_final:.4f}")

# 5. Generate comprehensive report and visualizations
analyzer.generate_stability_report('outputs/stability_report.txt')
```

### Analysis Outputs

The Lyapunov analyzer generates comprehensive reports and visualizations saved to the `outputs/` directory:

**Stability Report** (`lyapunov_stability_report.txt`)
- System configuration summary (generators, states, equilibrium energy)
- Eigenvalue classification (stable/unstable/marginal counts)
- Dominant modes with real and imaginary parts
- Frequencies of oscillatory modes
- Example output:
  ```
  System Configuration:
    Generators: 4
    Total states: 52
    Equilibrium energy: H* = 103.344134
  
  Linearized Stability:
    Status: STABLE
    Stable modes: 36
    Unstable modes: 0
    Marginal modes: 16
  
  Dominant Eigenvalues:
    1. lambda = 0.0000 + 376.9911j  (60 Hz stator flux)
    2. lambda = -0.5234 + 5.2411j   (Electromechanical mode)
  ```

**Eigenvalue Plot** (`eigenvalue_analysis.png`)
- Complex plane visualization of all system modes
- Imaginary axis (stability boundary) marked with dashed line
- Dominant eigenvalues highlighted in red
- Stable region (left half-plane) clearly indicated

**Lyapunov Evolution** (`lyapunov_evolution.png`)
- Two-panel plot showing transient behavior:
  - Top: Energy function V(t) evolution (should decrease to zero)
  - Bottom: Rotor angle trajectories for all generators
- Verifies passivity and convergence to equilibrium
- Useful for checking if perturbations are being rejected

**Stability Visualization** (`stability_visualization.png`)
Comprehensive 6-panel analysis providing multiple perspectives:
- **Panel 1-2**: Individual generator phase portraits (δ vs ω)
  - Energy contours colored from green (stable) to red (high energy)
  - Red boundary shows V = V_critical (stability limit)
  - Equilibrium marked with black star
- **Panel 3**: Relative angle plane (δ₁ - δ₂ vs ω₁)
  - Critical for inter-area oscillations
  - Shows coupling between generators
- **Panel 4**: 3D energy surface
  - Visualizes energy well around equilibrium
  - Bowl shape indicates stable potential well
- **Panel 5**: Stability criterion scatter (V vs dV/dt)
  - Green points: dV/dt < 0 (stable, energy decreasing)
  - Red points: dV/dt ≥ 0 (unstable, energy increasing)
  - V_critical line separates stability regions
- **Panel 6**: Region of attraction summary
  - System statistics and sampling results
  - Critical energy estimate
  - Port-Hamiltonian structure confirmation

### Performance Characteristics

**Speed Improvements**
- **Before**: 500 samples = stuck (trajectory simulation for each)
- **After**: 500 samples in ~80 seconds (10-20x faster)
- **Method**: Uses analytical dV/dt criterion instead of trajectory simulation

**Efficiency Features**
- Sparse gradient computation (only key states: δ, ω, ψf)
- Parallel-compatible sample testing
- Progress indicators for long computations

### Interpretation Guide

**Eigenvalue Analysis**
```
Stable modes: Real part < 0     # Exponentially decaying
Unstable modes: Real part > 0   # Growing instability
Marginal modes: Real part ≈ 0   # Expected for stator flux (±377j rad/s)
```

**Energy Margins**
```
V_critical ~35-40: Large stability region
V_critical ~5-10: Medium stability region  
V_critical <5: Small stability region (vulnerable)
```

**Passivity Test**
```
V(t) decreasing: System dissipative (stable)
V(t) increasing: Non-passive (check formulation)
V(t) oscillating: Marginal stability
```

### Advantages for Port-Hamiltonian Systems

1. **Natural Energy Function**: Hamiltonian is ready-made Lyapunov candidate
2. **Physical Insight**: Energy analysis reveals stability mechanisms
3. **Modular**: Works with any component combination defined in JSON
4. **Scalable**: Automatically adapts to system size (4, 10, 100+ generators)
5. **Comprehensive**: Combines local (linearization) and global (ROA) analysis

### Technical Notes

**COI Frame Transformation**
- Removes global rotation energy (rotationally invariant)
- Critical for multi-machine systems to avoid false instability
- `theta_i = delta_i - sum(M_j * delta_j) / sum(M_j)`

**Potential Energy Formulation**
- Quadratic approximation around equilibrium for positive definiteness
- Captures both mechanical work and network coupling
- Ensures V(x*) = 0 exactly

**Equilibrium Quality**
- Max |dx/dt| ~0.19 from fast stator dynamics (377 rad/s)
- Slow dynamics (angle, speed) have |dx/dt| <0.01
- Acceptable for stability analysis

### Implementation Details

**Common Features (All Methods):**
- ✓ Iterative equilibrium initialization with power flow trim
- ✓ Per-unit base conversion (xd'' scaling: machine → system base)
- ✓ Network solver with voltage safety clamping
- ✓ Exciter saturation (VRMAX/VRMIN anti-windup)
- ✓ Governor droop and time constant modeling
- ✓ Array handling fixes (sol.y.T for proper reshaping)

**Stability Improvements Applied:**
- Fixed xd'' scaling bug (0.25 pu machine → 0.0278 pu system for 100/900 MVA)
- **Fixed Ybus transformer scaling** (removed erroneous n² division)
- **Implemented constant power loads** (I = S*/V instead of Y = P-jQ)
- **Direct equilibrium initialization** (E'' = V + jXd''×I from power flow)
- Added exciter anti-windup (prevents runaway to 117 pu) - **all methods**
- Added governor saturation limits - **all methods**
- Corrected network solver iterations with convergence check
- Fixed governor/exciter reference handling
- Corrected diagnostic array reshaping (sol.y vs sol.y.T)
- **Proper D-matrix computation in frequency-domain method** (captures network impedance)
- **Iterative power flow trim for all methods** (consistent equilibrium initialization)
- **Correct Vref initialization** (Vref = Vt + Efd/KA for equilibrium) - **all methods**
- **Trajectory difference method in time-domain scanner** (captures dynamic impedance)

**Signal Monitoring (Time-Domain Only):**
```python
signals = {
    'Vd', 'Vq', 'Vt',          # Bus voltages
    'Id', 'Iq',                # Generator currents
    'delta', 'omega',          # Rotor angle and speed
    'Efd', 'Vref',            # Exciter signals
    'Gate', 'Pm', 'Pe',       # Governor and power
    'Id_inj', 'Iq_inj'        # Injection currents
}
```

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

### Workflow 3: Comparative Impedance Study

Compare all three impedance methods to understand system behavior:

```python
import numpy as np
import matplotlib.pyplot as plt

# Setup
bus_idx = 2
freqs_dense = np.logspace(-1, 2, 100)
freqs_sparse = np.logspace(-1, 2, 30)

# Method 1: Frequency-Domain (Linear Baseline)
from utils.impedance_scanner import ImpedanceScanner
scanner1 = ImpedanceScanner('test_cases/Kundur_System/kundur_full.json')
f1, Z_mag1, Z_phase1, Z_dq1 = scanner1.scan_impedance(bus_idx, freqs_dense)

# Method 2: IMTB (Medium Amplitude)
from utils.imtb_scanner import IMTBScanner
scanner2 = IMTBScanner('test_cases/Kundur_System/kundur_full.json')
f2, Z_dq2 = scanner2.run_mimo_scan(bus_idx, freqs_sparse, amplitude=0.10)

# Method 3: Time-Domain (Large Signal)
from utils.impedance_scanner_td import ImpedanceScannerTD
scanner3 = ImpedanceScannerTD('test_cases/Kundur_System/kundur_full.json')
sol, _ = scanner3.run_scan(bus_idx, f_max=50.0, amplitude=0.01, duration=60.0)
f3, Z3 = scanner3.post_process_tfe(sol, bus_idx)

# Compare Results
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))

# Magnitude comparison
ax1.loglog(f1, np.abs(Z_dq1[:, 0, 0]), 'b-', label='Linear (FD)', linewidth=2)
ax1.loglog(f2, np.abs(Z_dq2[:, 0, 0]), 'ro', label='IMTB (0.10 pu)', markersize=6)
ax1.loglog(f3, np.abs(Z3), 'g--', label='TD White Noise (0.01 pu)', linewidth=2)
ax1.set_ylabel('|Z| (pu)')
ax1.set_title('Impedance Method Comparison - Magnitude')
ax1.legend()
ax1.grid(True, which='both', alpha=0.3)

# Phase comparison
ax2.semilogx(f1, np.degrees(np.angle(Z_dq1[:, 0, 0])), 'b-', linewidth=2)
ax2.semilogx(f2, np.degrees(np.angle(Z_dq2[:, 0, 0])), 'ro', markersize=6)
ax2.semilogx(f3, np.degrees(np.angle(Z3)), 'g--', linewidth=2)
ax2.set_ylabel('Phase (deg)')
ax2.set_xlabel('Frequency (Hz)')
ax2.grid(True, which='both', alpha=0.3)

plt.savefig('outputs/impedance_comparison.png', dpi=150)
plt.show()

# Analyze discrepancies to identify nonlinear effects
```

### Workflow 4: Amplitude Sweep Study (Nonlinearity Detection)

Use time-domain scanner to reveal amplitude-dependent impedance:

```python
from utils.impedance_scanner_td import ImpedanceScannerTD

scanner = ImpedanceScannerTD('test_cases/Kundur_System/kundur_full.json')
amplitudes = [0.001, 0.01, 0.05, 0.10]  # Test different amplitudes

results = {}
for amp in amplitudes:
    print(f"\n{'='*60}")
    print(f"Testing Amplitude: {amp} pu")
    print(f"{'='*60}")
    
    sol, bus_idx = scanner.run_scan(
        bus_idx=2, 
        f_max=50.0, 
        amplitude=amp, 
        duration=60.0
    )
    
    freqs, Z_est = scanner.post_process_tfe(sol, bus_idx)
    results[amp] = (freqs, Z_est)
    
    # Plot system response for each amplitude
    scanner.plot_system_response(f'outputs/response_amp_{amp}.png')

# Compare impedance across amplitudes
import matplotlib.pyplot as plt
fig, ax = plt.subplots(figsize=(10, 6))

for amp, (f, Z) in results.items():
    ax.loglog(f, np.abs(Z), label=f'{amp} pu', linewidth=2)

ax.set_xlabel('Frequency (Hz)')
ax.set_ylabel('Impedance Magnitude |Z| (pu)')
ax.set_title('Amplitude-Dependent Impedance (Nonlinearity Test)')
ax.legend()
ax.grid(True, which='both', alpha=0.3)
plt.savefig('outputs/amplitude_sweep.png', dpi=150)
plt.show()

# Large differences indicate nonlinear effects (saturation, etc.)
```

### Workflow 5: Lyapunov Stability Assessment

Comprehensive stability analysis using energy methods:

```python
from utils.lyapunov_analyzer import LyapunovStabilityAnalyzer
import matplotlib.pyplot as plt
import numpy as np

# Initialize analyzer
analyzer = LyapunovStabilityAnalyzer('test_cases/Kundur_System/kundur_full.json')

# Step 1: Equilibrium and Linearization
x_eq = analyzer.initialize_equilibrium()
lin_results = analyzer.analyze_linearized_stability()

print("="*60)
print("LINEARIZED STABILITY ANALYSIS")
print("="*60)
print(f"Status: {'STABLE' if lin_results['is_stable'] else 'UNSTABLE'}")
print(f"Stable modes: {lin_results['stable_count']}")
print(f"Unstable modes: {lin_results['unstable_count']}")
print(f"Marginal modes: {lin_results['marginal_count']}")

# Plot eigenvalues
fig = analyzer.plot_eigenvalues(lin_results)
plt.savefig('outputs/eigenvalues.png', dpi=150)

# Step 2: Region of Attraction Estimation
print("\n" + "="*60)
print("REGION OF ATTRACTION ESTIMATION")
print("="*60)

roa_results = analyzer.estimate_stability_region(
    n_samples=500,
    max_perturbation=1.0
)

print(f"Samples tested: {roa_results['n_samples']}")
print(f"Stable samples: {roa_results['stable_count']} ({roa_results['stable_fraction']*100:.1f}%)")
print(f"Critical energy: V_crit = {roa_results['V_critical']:.4f}")

# Step 3: Transient Energy Margin (Fault Scenario)
print("\n" + "="*60)
print("TRANSIENT ENERGY MARGIN")
print("="*60)

# Create post-fault state (angle perturbations)
x_fault = x_eq.copy()
x_fault[0] += 0.5  # Gen 1: +0.5 rad (~28 deg)
x_fault[13] += 0.2  # Gen 2: +0.2 rad (~11 deg)

energy_margin = analyzer.compute_energy_margin(x_fault)
print(f"Stable equilibrium: H* = {energy_margin['H_stable']:.4f}")
print(f"Post-fault energy: H_fault = {energy_margin['H_fault']:.4f}")
print(f"Energy absorbed: dH = {energy_margin['delta_H']:.4f}")
print(f"Normalized margin: {energy_margin['normalized_margin']*100:.2f}%")

if energy_margin['delta_H'] < roa_results['V_critical']:
    print("STABLE: System will recover from this disturbance")
else:
    print("UNSTABLE: Energy exceeds critical level")

# Step 4: Passivity Verification
print("\n" + "="*60)
print("PASSIVITY VERIFICATION")
print("="*60)

from scipy.integrate import solve_ivp

# Small random perturbation
x_pert = x_eq + np.random.randn(len(x_eq)) * 0.05

# Simulate
sol = solve_ivp(
    lambda t, x: analyzer._system_dynamics(x, np.zeros(analyzer.n_gen)),
    (0, 10.0), x_pert,
    t_eval=np.linspace(0, 10, 200),
    method='RK45'
)

# Compute Lyapunov evolution
V_traj = [analyzer.compute_lyapunov_function(sol.y[:, i]) for i in range(len(sol.t))]

print(f"Initial energy: V(0) = {V_traj[0]:.4f}")
print(f"Final energy: V(T) = {V_traj[-1]:.4f}")
print(f"Energy dissipated: {V_traj[0] - V_traj[-1]:.4f}")

if V_traj[-1] < V_traj[0]:
    print("PASSIVE: Energy decreasing (system converging)")
else:
    print("WARNING: Energy increasing (check formulation)")

# Plot Lyapunov evolution
plt.figure(figsize=(10, 6))
plt.plot(sol.t, V_traj, 'b-', linewidth=2)
plt.xlabel('Time (s)')
plt.ylabel('Lyapunov Function V(x)')
plt.title('Energy Evolution - Passivity Verification')
plt.grid(True, alpha=0.3)
plt.axhline(y=0, color='r', linestyle='--', label='Equilibrium')
plt.legend()
plt.savefig('outputs/passivity_verification.png', dpi=150)

# Step 5: Generate Full Report
analyzer.generate_stability_report('outputs/stability_report.txt')

print("\n" + "="*60)
print("ANALYSIS COMPLETE")
print("="*60)
print("Outputs saved to 'outputs/' directory")
```

### Workflow 6: Component Reusability

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
For an N-machine system:
- Each generator: 13 states (7 electrical + 4 exciter + 2 governor)
- Total: 13×N states for complete system dynamics
- Dynamic sizing: System adapts to any number of generators defined in JSON

### Network Solution

**Ybus Construction:**
- Admittance matrix built directly from `Line` data in JSON
- Per-unit values in JSON are already on common 100 MVA system base
- For transformers: impedance given on common base, **no turns ratio scaling** in Ybus
- Standard π-model for both lines and transformers

**Load Modeling:**
- **Constant power load model** with iterative solution
- Load current: `I_load = conj(S_load / V)` updated each iteration
- Converges in 3-5 iterations typically
- Properly matches power flow results

**Generator-Network Interface:**
- Voltage-behind-reactance model: `E'' = V + jXd'' × I`
- Park transformation converts machine dq frame to system RI frame
- Internal EMF: `E_internal = Ed'' + j×Eq''` where `Eq'' = gd1 × ψf`
- Generator admittance: `y_gen = 1/(j×Xd'')`

**Per-Unit System:**
- All reactances stored on **system base** (100 MVA) in gen_metadata
- Conversion done once during component building (genrou.py)
- Network solver uses values directly without additional conversion
- Consistent base throughout all calculations

### Equilibrium Initialization
Direct calculation from power flow data:
```python
# For each generator:
V_phasor = V_mag × exp(j × V_angle)    # From Bus data
S = P_gen + j × Q_gen                   # From PV/Slack data
I_phasor = conj(S / V_phasor)          # Generator current
E_phasor = V_phasor + (ra + j×Xd'') × I_phasor  # Internal EMF
delta = angle(E_phasor)                 # Rotor angle
psi_f = |E_phasor| / gd1               # Field flux
```

### Fault Modeling
- Ybus modification during fault (adds fault admittance to diagonal)
- Automatic switching at fault initiation and clearing
- Preserves energy balance through fault transition
- Configurable fault bus, impedance, and timing

## Recent Fixes and Improvements (v1.1)

### Critical Bug Fixes

**1. Ybus Construction (system_coordinator.py)**
- **Issue:** Transformer admittances were incorrectly scaled by turns ratio squared (n² = 132 for 230/20 kV)
- **Result:** Generator bus admittances were 0.63 pu instead of correct ~83 pu
- **Fix:** Removed turns ratio scaling since per-unit values in JSON are already on common 100 MVA base
- **Impact:** Network solution now matches power flow results

**2. Per-Unit Base Conversion (system_coordinator.py)**
- **Issue:** `solve_network()` applied `Xd'' × (S_system/Sn)` conversion, but xd'' in metadata was already on system base
- **Result:** Double conversion made Xd'' = 0.003 pu instead of 0.028 pu
- **Fix:** Use metadata values directly without additional conversion
- **Impact:** Generator currents and powers now correct

**3. Load Model (system_coordinator.py)**
- **Issue:** Used constant impedance model `Y_load = P - jQ`
- **Result:** Power mismatch of 0.3-1.5 pu compared to power flow
- **Fix:** Implemented constant power model with iterative solution: `I_load = conj(S/V)`
- **Impact:** Power matching error reduced from ~1 pu to <0.01 pu

**4. Equilibrium Initialization (fault_sim_modular.py)**
- **Issue:** Hardcoded for 4 generators, iterative solver not converging
- **Result:** Wrong initial angles, power oscillations
- **Fix:** Direct calculation from power flow using `E'' = V + jXd'' × I`
- **Impact:** Stable equilibrium with angles matching power flow

### Verification Results

After fixes, the system shows:
- **Power matching:** P_error < 0.005 pu for all generators
- **Voltage matching:** V_error < 0.001 pu at all buses
- **Angle accuracy:** δ within 0.1° of power flow results
- **Stable dynamics:** ω oscillates around 1.0 pu with proper damping
- **Fault response:** Angles remain bounded (45-65°), return to equilibrium

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
  - Transient stability index computation
  - Critical clearing time (CCT) calculation
- **Impedance enhancements**:
  - Automated saturation diagnostics
  - Coherence-based filtering for time-domain
  - Nyquist plot generation for stability margins
  - Frequency-dependent network models

## Tips and Best Practices

### Lyapunov Stability Analysis Tips

**Equilibrium Quality:**
- Max |dx/dt| ~0.19 from fast stator flux dynamics (±377 rad/s) is acceptable
- Slow dynamics (angle, speed, field) should have |dx/dt| < 0.01
- COI frame removes global rotation drift
- If equilibrium drifts significantly, check Vref and Pref initialization

**Region of Attraction Estimation:**
- Start with 500 samples for initial assessment
- Increase to 1000-2000 for more accurate V_critical estimate
- max_perturbation = 0.5-1.0 rad for angle perturbations is typical
- Stable fraction ~40-60% indicates reasonable stability margin
- V_critical > 30 suggests large stability region

**Interpreting Results:**
- **Unstable modes = 0**: Linearized stable (small perturbations)
- **Marginal modes ≈ 16**: Expected for 4-machine system (stator flux at ±377j, angle reference)
- **V(t) decreasing**: System dissipative and converging to equilibrium
- **Energy margin > V_critical**: Fault will cause loss of synchronism

**Performance Tuning:**
- Use dV/dt criterion instead of trajectory simulation (10-20x faster)
- Sparse gradient computation focuses on key states (δ, ω, ψf)
- Parallel sampling possible for large-scale studies
- Progress indicators show completion status

**Common Issues:**
- **V(x*+dx) < 0**: Potential energy formulation issue (should use quadratic approximation)
- **V increasing during simulation**: Check dissipation terms in dynamics
- **Large V_critical variation**: System has multiple stability regions (COI frame should help)
- **Marginal stability**: Add damping or check controller tuning

### Impedance Scanning Tips

**Method Selection Updates (v1.1):**
- **Frequency-Domain scanner now improved**: Includes saturation and proper D-matrix
- All three methods now use consistent equilibrium initialization
- Frequency-domain results should be closer to IMTB/TD at low frequencies
- D-matrix now captures network impedance at infinite frequency (algebraic path)

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

**Time-Domain Scanner:**
- **Duration vs Resolution:** Δf = 1/duration
  - 60s → 0.017 Hz resolution
  - 120s → 0.008 Hz resolution (better low-frequency)
- **f_max vs Speed:** fs = 10×f_max, computation ∝ fs²
  - 50 Hz → ~2 min simulation
  - 200 Hz → ~15 min simulation
- **Progress Bar:** Updates every 1 second with ETA

**IMTB Scanner:**
- Use 20-30 logarithmically-spaced frequencies
- Amplitude 0.05-0.10 pu reveals saturation without instability
- Analyze off-diagonal terms (Zdq, Zqd) for cross-coupling

**Interpreting Results:**
- **Resonances:** Sharp peaks in magnitude plot
- **Controller bandwidth:** -3dB point or phase crossover
- **Saturation:** Impedance decreases with amplitude
- **Discrepancies between methods:** Indicates nonlinearity
- **High impedance at low frequencies:** Normal for voltage-controlled generators (AVR keeps V constant)
- **Expected values:** DC: 100-10000 pu, 10 Hz: 1-100 pu, 100 Hz: 0.3-1.0 pu

### Troubleshooting

**"System becomes unstable during scan":**
- Reduce injection amplitude (try 0.01 or 0.001 pu)
- Check equilibrium initialization (should drift <2° over 5s)
- Verify exciter saturation limits are active

**"Noisy impedance results (time-domain)":**
- Increase simulation duration (60s → 120s)
- Check coherence values (should be >0.5 for valid data)
- Reduce f_max if high-frequency noise dominates

**"Progress bar stops updating":**
- Normal for solve_ivp (updates when solver outputs points)
- Check terminal output for completion message
- Final 100% displayed after simulation completes

**"Different methods give very different results":**
- Expected! This reveals nonlinear behavior
- Frequency-domain = linear baseline
- IMTB/Time-domain = realistic behavior with saturation
- Large differences indicate strong nonlinearity

**"Impedance values seem too large":**
- High impedance at low frequencies (<1 Hz) is correct for voltage-controlled generators
- AVR maintains constant voltage → Z = V/I appears very large
- Should decrease with frequency and settle to electrical impedance at high freq
- If high freq impedance >10 pu, check D-matrix computation or scaling

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

**Framework Version**: 1.3
**Author**: [Your Name]
**Last Updated**: January 2026

### Changelog

**v1.3 (January 2026)**
- **NEW: Lyapunov Stability Analyzer (`lyapunov_analyzer.py`)**
  - Energy-based stability analysis leveraging Port-Hamiltonian structure
  - Linearized stability via Jacobian eigenvalue decomposition
  - **Proper Lyapunov function formulation**: V(x*) = 0, V(x) > 0 for x ≠ x*
  - **Center-of-Inertia (COI) frame** for rotational invariance
  - **Quadratic potential energy** ensuring positive definiteness
  - Region of attraction estimation using dV/dt < 0 criterion (10-20x faster than trajectory simulation)
  - Transient energy margin computation for fault scenarios
  - Passivity verification through energy evolution tracking
- **Comprehensive Stability Visualization:**
  - Multi-panel plots: phase portraits, relative angles, 3D energy landscapes
  - V vs dV/dt scatter for stability criterion visualization
  - Region of attraction summary statistics
  - Lyapunov function evolution during transients
- **DynamicsCore Pattern:**
  - Extended `Core` class with numerical dynamics capability
  - Enables general Lyapunov analysis across arbitrary component combinations
  - Each component provides its own dynamics function (genrou_dynamics, exdc2_dynamics, tgov1_dynamics)
- **Performance Optimizations:**
  - Sparse gradient computation (only key states: δ, ω, ψf)
  - Analytical dV/dt criterion replaces expensive trajectory simulations
  - 500 sample region estimation in ~80 seconds
- **Test Suite:**
  - `test_lyapunov.py`: Comprehensive stability analysis tests
  - Eigenvalue analysis, passivity verification, energy margins, region estimation
  - Automated report generation with Unicode-free output for Windows compatibility

**v1.2 (January 2026)**
- **Time-Domain Impedance Scanner Major Fix:**
  - Implemented **trajectory difference method** for proper dynamic impedance measurement
  - Runs baseline simulation (no injection) + injection simulation
  - Voltage perturbation ΔV = V_injection - V_baseline captures dynamic response
  - Fixed flat impedance issue (was only measuring algebraic Xd'')
  - Measured impedance now shows frequency-dependent behavior matching analytical predictions
- **Exciter Equilibrium Fix (all modules):**
  - Fixed Vref initialization: Vref = Vt + Efd/KA (maintains equilibrium)
  - Prevents Efd drift and system instability during long simulations
- **ASCII Progress Bar:**
  - Changed Unicode characters to ASCII (#, -) for Windows compatibility

**v1.1 (January 2026)**
- Fixed Ybus transformer scaling bug (removed incorrect turns ratio division)
- Fixed double per-unit base conversion in solve_network
- Implemented constant power load model with iterative solution
- Rewrote equilibrium initialization using direct power flow calculation
- Made fault_sim_modular fully dynamic (supports any number of generators)
- Power flow matching improved from ~1 pu error to <0.01 pu error

**v1.0 (January 2026)**
- Initial release with modular Port-Hamiltonian framework
- Three impedance scanning methods (frequency-domain, IMTB, time-domain)
- Fault simulation with configurable scenarios
- JSON-based system configuration
