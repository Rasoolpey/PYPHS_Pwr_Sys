# JSON Configuration Guide

This guide explains the JSON configuration format for defining power systems and simulations.

## Two-File Architecture

The framework uses **two separate JSON files** for clean separation of concerns:

1. **System JSON**: Physical system parameters (buses, lines, generators, etc.)
2. **Simulation JSON**: Runtime parameters (time, fault, solver settings)

### Example Usage

```python
from utils.fault_sim_modular import ModularFaultSimulator

# Initialize with both configs
sim = ModularFaultSimulator(
    system_json='test_cases/Thevenin_model/ieee_based_system.json',
    simulation_json='test_cases/Thevenin_model/simulation_fault.json'
)

# Run simulation
x0 = sim.initialize_equilibrium()
sol = sim.simulate(x0, t_end=sim.t_end)
```

## System JSON Format

### Required Sections

#### Bus
Defines network buses.

```json
{
  "Bus": [
    {
      "idx": 1,
      "u": 1.0,
      "name": "Bus1",
      "Vn": 20.0,
      "vmax": 1.1,
      "vmin": 0.9,
      "v0": 1.0,
      "a0": 0.0,
      "area": 1,
      "zone": 1,
      "owner": 1
    }
  ]
}
```

**Fields:**
- `idx` (int): Bus index (unique identifier)
- `u` (float): Status (1.0 = in service, 0.0 = out of service)
- `name` (str): Bus name
- `Vn` (float): Nominal voltage (kV)
- `v0` (float): Initial voltage magnitude (pu)
- `a0` (float): Initial voltage angle (radians)
- `vmax`, `vmin` (float): Voltage limits (pu)
- `area`, `zone`, `owner` (int): Area/zone/owner indices

#### Line
Defines transmission lines and transformers.

```json
{
  "Line": [
    {
      "idx": 1,
      "u": 1.0,
      "name": "Line_1_2",
      "bus1": 1,
      "bus2": 2,
      "Sn": 100.0,
      "fn": 60.0,
      "Vn1": 20.0,
      "Vn2": 345.0,
      "r": 0.0,
      "x": 0.1,
      "b": 0.0,
      "trans": 1,
      "tap": 1.0
    }
  ]
}
```

**Fields:**
- `idx` (int): Line index
- `bus1`, `bus2` (int): From/to bus indices
- `Sn` (float): Power base (MVA)
- `fn` (float): Frequency (Hz)
- `Vn1`, `Vn2` (float): Voltage bases (kV)
- `r` (float): Resistance (pu)
- `x` (float): Reactance (pu)
- `b` (float): Shunt susceptance (pu)
- `trans` (int): 1 if transformer, 0 if line
- `tap` (float): Transformer tap ratio

#### PQ
Defines constant power loads.

```json
{
  "PQ": [
    {
      "idx": 1,
      "u": 1.0,
      "name": "Load_1",
      "bus": 5,
      "Vn": 230.0,
      "p0": 10.0,
      "q0": 3.0,
      "vmax": 1.1,
      "vmin": 0.9
    }
  ]
}
```

**Fields:**
- `idx` (int): Load index
- `bus` (int): Bus where load is connected
- `p0` (float): Active power (pu on system base)
- `q0` (float): Reactive power (pu on system base)

#### PV
Defines PV (voltage-controlled) buses.

```json
{
  "PV": [
    {
      "idx": 1,
      "u": 1.0,
      "name": "Gen_1",
      "bus": 1,
      "Sn": 900.0,
      "Vn": 20.0,
      "p0": 7.0,
      "q0": 0.0,
      "v0": 1.0,
      "pmax": 10.0,
      "pmin": 0.0,
      "qmax": 5.0,
      "qmin": -5.0
    }
  ]
}
```

**Fields:**
- `bus` (int): Bus where generator is connected
- `Sn` (float): Generator rating (MVA)
- `p0`, `q0` (float): Initial power output (pu)
- `v0` (float): Voltage setpoint (pu)

#### Slack
Defines slack/reference bus.

```json
{
  "Slack": [
    {
      "idx": 1,
      "u": 1.0,
      "name": "Grid",
      "bus": 3,
      "Sn": 100.0,
      "Vn": 345.0,
      "v0": 1.0,
      "a0": 0.0,
      "p0": 0.0,
      "q0": 0.0
    }
  ]
}
```

**Fields:**
- `bus` (int): Slack bus index
- `v0` (float): Voltage magnitude (pu)
- `a0` (float): Voltage angle (radians)
- `p0`, `q0` (float): Power injection (calculated by power flow)

### Component Sections

#### GENROU (Generator)
Round-rotor synchronous generator model (6th order).

```json
{
  "GENROU": [
    {
      "idx": 1,
      "u": 1.0,
      "name": "Gen_1",
      "bus": 1,
      "gen": 1,
      "Sn": 900.0,
      "Vn": 20.0,
      "fn": 60.0,
      "M": 13.0,
      "D": 0.0,
      "ra": 0.0025,
      "xl": 0.15,
      "xd": 1.8,
      "xq": 1.7,
      "xd1": 0.3,
      "xq1": 0.55,
      "xd2": 0.25,
      "xq2": 0.25,
      "Td10": 8.0,
      "Td20": 0.03,
      "Tq10": 0.4,
      "Tq20": 0.05
    }
  ]
}
```

**Key Fields:**
- `bus` (int): Bus where generator is connected
- `gen` (int): Reference to PV/Slack entry
- `Sn` (float): Machine rating (MVA)
- `M` (float): Inertia constant (s)
- `D` (float): Damping coefficient
- `ra` (float): Armature resistance (pu)
- `xl` (float): Leakage reactance (pu)
- `xd`, `xq` (float): d/q-axis synchronous reactances (pu)
- `xd1`, `xq1` (float): d/q-axis transient reactances (pu)
- `xd2`, `xq2` (float): d/q-axis subtransient reactances (pu)
- `Td10`, `Tq10` (float): d/q-axis transient time constants (s)
- `Td20`, `Tq20` (float): d/q-axis subtransient time constants (s)

#### ESST3A (Exciter)
IEEE ST3A static excitation system.

```json
{
  "ESST3A": [
    {
      "idx": 1,
      "u": 1.0,
      "name": "Exc_1",
      "syn": 1,
      "TR": 0.02,
      "KA": 20.0,
      "TA": 0.02,
      "TC": 1.0,
      "TB": 5.0,
      "KM": 8.0,
      "TM": 0.4,
      "VRMAX": 99.0,
      "VRMIN": -99.0,
      "VMMAX": 99.0,
      "VMMIN": 0.0,
      "VIMAX": 0.2,
      "VIMIN": -0.2,
      "VBMAX": 5.48,
      "KC": 0.01,
      "KP": 3.67,
      "KI": 0.435,
      "XL": 0.0098,
      "KG": 1.0,
      "VGMAX": 3.86,
      "THETAP": 3.33
    }
  ]
}
```

**Key Fields:**
- `syn` (int): References generator `idx`
- `TR` (float): Voltage transducer time constant (s)
- `KA`, `TA` (float): Voltage regulator gain and time constant
- `TC`, `TB` (float): Lead-lag time constants (s)
- `KM`, `TM` (float): Inner regulator gain and time constant
- `VRMAX`, `VRMIN` (float): Voltage regulator limits
- `KC`, `KP`, `KI`, `XL` (float): Rectifier compensation parameters
- `THETAP` (float): Compensation angle (degrees)

#### EXDC2 (Exciter)
DC exciter model.

```json
{
  "EXDC2": [
    {
      "idx": 1,
      "u": 1.0,
      "name": "Exc_1",
      "syn": 1,
      "TR": 0.01,
      "KA": 40.0,
      "TA": 0.02,
      "KE": 1.0,
      "TE": 0.5,
      "KF1": 0.08,
      "TF1": 1.0,
      "TC": 0.0,
      "TB": 0.0,
      "VRMAX": 4.0,
      "VRMIN": -4.0,
      "E1": 2.85,
      "SE1": 0.38,
      "E2": 3.8,
      "SE2": 0.85
    }
  ]
}
```

**Key Fields:**
- `syn` (int): References generator `idx`
- `TR`, `TA`, `TE`, `TF1` (float): Time constants (s)
- `KA`, `KE`, `KF1` (float): Gains
- `VRMAX`, `VRMIN` (float): Regulator limits
- `E1`, `SE1`, `E2`, `SE2` (float): Saturation curve parameters

#### TGOV1 (Governor)
Steam turbine governor model.

```json
{
  "TGOV1": [
    {
      "idx": 1,
      "u": 1.0,
      "name": "Gov_1",
      "syn": 1,
      "wref0": 1.0,
      "R": 0.05,
      "VMAX": 1.2,
      "VMIN": 0.0,
      "T1": 0.1,
      "T2": 0.5,
      "T3": 3.0,
      "Dt": 0.0
    }
  ]
}
```

**Key Fields:**
- `syn` (int): References generator `idx`
- `R` (float): Droop (pu)
- `VMAX`, `VMIN` (float): Valve limits
- `T1`, `T2`, `T3` (float): Time constants (s)
- `Dt` (float): Turbine damping coefficient

## Simulation JSON Format

Defines runtime parameters for simulations.

```json
{
  "description": "Fault simulation configuration",
  "time": {
    "t_end": 15.0,
    "n_points": 3000
  },
  "fault": {
    "enabled": true,
    "bus": 2,
    "impedance_real": 0.0,
    "impedance_imag": 0.005,
    "start_time": 1.0,
    "duration": 0.1
  },
  "solver": {
    "method": "Radau",
    "rtol": 1e-6,
    "atol": 1e-8
  },
  "output": {
    "directory": "outputs",
    "save_states": true,
    "plot_format": "png"
  }
}
```

### Sections

#### time
Simulation time parameters.

**Fields:**
- `t_end` (float): End time (seconds)
- `n_points` (int): Number of time points for output

#### fault
Fault configuration.

**Fields:**
- `enabled` (bool): Enable/disable fault
- `bus` (int): Bus where fault occurs
- `impedance_real` (float): Fault resistance (pu)
- `impedance_imag` (float): Fault reactance (pu)
- `start_time` (float): Fault start time (seconds)
- `duration` (float): Fault duration (seconds)

#### solver
ODE solver settings.

**Fields:**
- `method` (str): Solver method ('Radau', 'BDF', 'LSODA', etc.)
- `rtol` (float): Relative tolerance
- `atol` (float): Absolute tolerance

#### initialization
Initial condition computation settings.

**Fields:**
- `run_power_flow` (bool): Automatically solve AC power flow before RMS simulation
- `power_flow_verbose` (bool): Print detailed power flow results

**Example:**
```json
{
  "initialization": {
    "run_power_flow": true,
    "power_flow_verbose": true
  }
}
```

**When to use power flow:**
- When initial conditions in system JSON are approximate or unknown
- For large multi-machine systems where manual initialization is difficult
- To ensure consistent voltage magnitudes, angles, and power flows
- To reduce initial transients in dynamic simulation

**Note:** For small systems with accurate JSON initial conditions, power flow may not be necessary.

#### output
Output configuration.

**Fields:**
- `directory` (str): Output directory path
- `save_states` (bool): Save state trajectories
- `plot_format` (str): Plot format ('png', 'svg', 'pdf')

## Power Flow Initialization

The framework includes a general AC power flow solver for finding accurate initial conditions.

### Why Use Power Flow?

In RMS (Root-Mean-Square) simulations, accurate initial conditions are critical. Problems arise when:
- Initial voltages/angles in JSON are approximate guesses
- Generator power outputs don't match load demand
- System is not in steady-state equilibrium
- Initial transients obscure actual dynamic behavior

### How It Works

The power flow solver:
1. Classifies buses as Slack (reference), PV (voltage-controlled), or PQ (load)
2. Solves nonlinear AC power flow equations using Newton-Raphson method
3. Computes consistent voltage magnitudes and angles at all buses
4. Calculates generator power outputs and line flows
5. Updates system JSON data with accurate initial conditions
6. Provides proper initial states for dynamic simulation

### Usage Methods

**Method 1: Automatic (via simulation JSON)**

```json
{
  "initialization": {
    "run_power_flow": true,
    "power_flow_verbose": true
  }
}
```

The simulator automatically solves power flow before initializing equilibrium.

**Method 2: Manual (in Python script)**

```python
from utils.fault_sim_modular import ModularFaultSimulator

sim = ModularFaultSimulator(
    system_json='system.json',
    simulation_json='simulation.json'
)

# Solve power flow first
sim.solve_power_flow(verbose=True)

# Then initialize equilibrium
x0 = sim.initialize_equilibrium()
```

**Method 3: Direct Power Flow**

```python
from utils.system_builder import PowerSystemBuilder
from utils.system_coordinator import PowerSystemCoordinator
from utils.power_flow import PowerFlowSolver

# Build system
builder = PowerSystemBuilder('system.json')
builder.build_all_components()

# Create coordinator
coordinator = PowerSystemCoordinator(builder)

# Solve power flow
pf = PowerFlowSolver(builder, coordinator)
converged = pf.solve(verbose=True)

if converged:
    pf.print_results()
    pf.update_system_data()
```

### Power Flow Features

- **General**: Works with any system topology from PowerSystemBuilder
- **Robust**: Newton-Raphson with numerical safeguards
- **Flexible**: Supports PQ, PV, and Slack buses
- **Integrated**: Seamlessly updates system data for RMS simulation
- **Informative**: Detailed convergence and results output

### When Power Flow May Not Converge

Power flow may fail to converge if:
- System is heavily loaded or near voltage collapse
- Initial guess is very far from solution
- Network data has errors (wrong impedances, missing connections)
- Bus types are incorrectly classified

In such cases, the simulator continues with JSON initial conditions.

## Multiple Scenarios

Create multiple simulation JSONs for different scenarios:

```
test_cases/
  MySystem/
    system.json                      # Physical system
    simulation_fault.json            # Fault scenario
    simulation_nofault.json          # Stability test
    simulation_with_powerflow.json   # With power flow initialization
```

## Example: Complete Test Case

### System JSON (`ieee_based_system.json`)
```json
{
  "Bus": [...],
  "Line": [...],
  "PQ": [...],
  "PV": [...],
  "Slack": [...],
  "GENROU": [...],
  "ESST3A": [...],
  "TGOV1": [...]
}
```

### Simulation JSON (`simulation_fault.json`)
```json
{
  "time": {"t_end": 15.0, "n_points": 3000},
  "fault": {"enabled": true, "bus": 2, "start_time": 1.0, "duration": 0.1},
  "solver": {"method": "Radau", "rtol": 1e-6, "atol": 1e-8}
}
```

### Test Script
```python
from utils.fault_sim_modular import ModularFaultSimulator

sim = ModularFaultSimulator(
    system_json='test_cases/MySystem/system.json',
    simulation_json='test_cases/MySystem/simulation_fault.json'
)

x0 = sim.initialize_equilibrium()
sol = sim.simulate(x0, t_end=sim.t_end)
sim.plot_results(sol, filename='my_simulation.png')
```

## Component Swapping

To swap components, simply change the component section name and parameters:

**Before (EXDC2 exciter):**
```json
{
  "EXDC2": [{"idx": 1, "syn": 1, ...}]
}
```

**After (ESST3A exciter):**
```json
{
  "ESST3A": [{"idx": 1, "syn": 1, ...}]
}
```

No code changes required! The framework automatically detects and loads the new component type.

## Validation Tips

1. **Check indices**: Ensure all `idx` fields are unique within their section
2. **Verify references**: `syn` fields must match generator `idx` values
3. **Bus connectivity**: All `bus` references must exist in the `Bus` section
4. **Parameter ranges**: Check that parameters are within physically reasonable ranges
5. **Base consistency**: Ensure `Sn` (machine base) and system base are compatible

## Common Issues

### Issue: Component Not Found
**Cause**: Component type not registered in factory  
**Solution**: Add component to `utils/component_factory.py` registry

### Issue: Index Mismatch
**Cause**: `syn` field doesn't match any generator  
**Solution**: Verify generator `idx` matches exciter/governor `syn`

### Issue: Unrealistic Results
**Cause**: Incorrect parameter values or units  
**Solution**: Check parameter per-unit bases and scaling

## Best Practices

1. **Use descriptive names**: Name components clearly (e.g., "Gen_Area1", "Exc_Unit2")
2. **Document systems**: Add comments in JSON (unofficial but helpful)
3. **Version control**: Track changes to system configurations
4. **Test incrementally**: Start simple, add complexity gradually
5. **Validate parameters**: Cross-check with manufacturer data or standards
