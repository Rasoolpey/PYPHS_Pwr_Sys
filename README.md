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

### 3. Perform Impedance Scanning

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

### 4. Run Test Scripts

```bash
python main.py                      # Build and verify system
python test_system.py               # Test individual components
python test_modular_fault.py        # Test fault simulation
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
| **Technique** | Linearization | Multisine DFT | Welch's PSD Estimation |
| **Computation** | State-space analysis | Time simulation | Time simulation |
| **Speed** | Very Fast (~seconds) | Medium (~minutes) | Slow (~minutes) |
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

**Principle:** Injects band-limited white noise and uses spectral analysis (Welch's method) to estimate the transfer function from current to voltage.

**Mathematical Approach:**
1. Inject band-limited white noise: I(t) ~ N(0, σ²), filtered to [0, f_max]
2. Measure voltage response V(t)
3. Compute PSDs: Z(f) = Pᵥᵢ(f) / Pᵢᵢ(f)

**Key Features:**
- ✓ Captures full nonlinear dynamics and saturation
- ✓ Most realistic method (simulates actual disturbances)
- ✓ Amplitude-dependent impedance (reveals nonlinearity)
- ✓ Visual progress bar with real-time ETA
- ✓ Comprehensive signal extraction (13 signal types)
- ✓ 8-panel system response visualization
- ⚠ Computationally expensive (60s simulation takes ~2-5 minutes)
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

**Advanced Features:**
- **Progress Tracking:** Real-time progress bar showing elapsed/total time and ETA
- **Signal Extraction:** Captures Vd, Vq, Id, Iq, δ, ω, Efd, Gate, Pm, Pe, Vref
- **Response Plotting:** 8-panel visualization of system dynamics
- **Saturation Diagnostics:** Can identify when/where controllers saturate

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
- Added exciter anti-windup (prevents runaway to 117 pu) - **all methods**
- Added governor saturation limits - **all methods**
- Corrected network solver iterations (3 → 2 with safety check)
- Fixed governor/exciter reference handling
- Corrected diagnostic array reshaping (sol.y vs sol.y.T)
- **Proper D-matrix computation in frequency-domain method** (captures network impedance)
- **Iterative power flow trim for all methods** (consistent equilibrium initialization)

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

### Workflow 5: Component Reusability

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
- **Impedance enhancements**:
  - Automated saturation diagnostics
  - Coherence-based filtering for time-domain
  - Nyquist plot generation for stability margins
  - Frequency-dependent network models

## Tips and Best Practices

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

**Framework Version**: 1.0  
**Author**: [Your Name]  
**Last Updated**: January 2026
