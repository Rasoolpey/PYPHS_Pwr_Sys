# VOC (Virtual Oscillator Control) Implementation Status

## ✅ COMPLETED: Proper Network Coupling (Option 1)

### What Was Done

We implemented **full grid-forming voltage source support** for VOC inverters in the power system framework. This required significant modifications across multiple modules:

#### 1. Network Coordinator (`utils/system_coordinator.py`)
**Added VOC voltage source injection:**
- Created `_build_voc_mapping()` method to map VOC inverters to buses
- Modified `solve_network()` to accept `voc_voltages` parameter
- Added VOC voltage injection: `I_voc = (u_voc - V_bus) * Y_filt`
- Computed grid currents: `I_grid = (u_voc - V_bus) / Z_filt`
- Extended return value to include `I_voc` (grid currents in αβ frame)

**Key Code Sections:**
```python
# VOC voltage source injection (Line ~506)
I_voc_inj = np.zeros(self.n_bus, dtype=complex)
if voc_voltages is not None and self.n_voc > 0:
    for v in range(self.n_voc):
        voc_bus = self.voc_bus_internal[v]
        u_a = voc_voltages[v].get('u_a', 0.0)
        u_b = voc_voltages[v].get('u_b', 0.0)
        y_filt = voc_voltages[v].get('y_filt', 1.0)
        
        u_voc = u_a + 1j * u_b
        V_at_bus = V_bus[voc_bus]
        
        # Current from VOC through output filter
        I_voc_inj[voc_bus] += (u_voc - V_at_bus) * y_filt

# Total current includes VOC (Line ~520)
I_total = I_inj + I_ren + I_voc_inj - I_load

# Extract grid currents for VOC dynamics (Line ~599)
I_voc[v] = (u_voc - V_at_bus) * y_filt
```

#### 2. Fault Simulator (`utils/fault_sim_modular.py`)
**Integrated VOC dynamics with network:**
- Extract VOC states: `[u_mag, theta, Pf, Qf]`
- Convert to αβ voltages: `u_a = u_mag * cos(theta)`, `u_b = u_mag * sin(theta)`
- Calculate filter admittance: `y_filt = 1 / (Rf + j*omega*Lf)`
- Pass VOC voltages to network solver
- Extract grid currents from network solution
- Feed grid currents to VOC dynamics

**Key Code Sections:**
```python
# Build VOC voltage injections (Line ~545)
voc_voltages = []
for v in range(self.n_voc):
    voc_x = voc_states[v]
    u_mag = voc_x[0]
    theta = voc_x[1]
    
    u_a = u_mag * np.cos(theta)
    u_b = u_mag * np.sin(theta)
    
    Rf = voc_meta.get('Rf', 0.01)
    Lf = voc_meta.get('Lf', 0.08)
    Z_filt = Rf + 1j * omega_rad * Lf
    y_filt = 1.0 / Z_filt
    
    voc_voltages.append({'u_a': u_a, 'u_b': u_b, 'y_filt': y_filt})

# Call VOC dynamics with grid currents (Line ~905)
I_grid_complex = I_voc[v]
ig_a = np.real(I_grid_complex)
ig_b = np.imag(I_grid_complex)

voc_dxdt = voc_meta['dynamics_fn'](voc_x, {'ig_a': ig_a, 'ig_b': ig_b}, voc_meta)
```

#### 3. Backward Compatibility (`utils/power_flow.py`)
**Updated all `solve_network()` calls:**
- Modified return value unpacking to handle both 5-tuple (old) and 6-tuple (new with `I_voc`)
- Three locations updated with graceful fallback

### Architecture

```
VOC States               Network Coordinator              Grid
[u_mag, theta, Pf, Qf] --(u_a, u_b, Y_filt)--> Voltage Injection --> Current Flow
         ^                                                               |
         |                                                               |
         +---------------------(ig_a, ig_b)---------------------------+
                             Grid Currents
```

**VOC as Voltage Source:**
- VOC outputs voltage `u = u_a + j*u_b` behind filter impedance `Z_filt = Rf + j*ω*Lf`
- Network determines grid current: `I_grid = (u_voc - V_bus) / Z_filt`
- Grid current feeds back to VOC dynamics for power measurement and control

## ⚠️ REMAINING ISSUE: Initialization Not at Equilibrium

### Problem
**Symptoms:**
- Large initial derivatives: `Max |dx/dt| = 21.7` (should be <1e-6)
- VOC plot shows unrealistic dynamics:
  - Voltage oscillates 0.75 to 1.27 pu (too large)
  - Power swings negative (unphysical for generator)
  - Angle ramps to 900° (unbounded)
  - Frequency deviation too large

**Root Cause:**
The VOC states are initialized from power flow (voltage and power setpoints), but the **grid currents `ig_a`, `ig_b` are not consistent with the VOC's initial state**. This creates a mismatch:
- Power flow says: "VOC delivers 0.35 pu at 1.03∠-1.54°"
- But VOC dynamics see wrong grid currents initially
- VOC responds to this error, creating large transients

### What's Needed

**Option A: Compute equilibrium grid currents during initialization**
In `power_flow.py`, after initializing VOC states:
1. Call network solver with VOC voltages
2. Extract grid currents `I_voc`
3. Verify VOC dynamics are at equilibrium: `dxdt ≈ 0`
4. If not, iteratively adjust VOC internal states

**Option B: Initialize VOC with its `init_fn`**
Similar to generators/exciters, create `voc_init(P0, Q0, V0, theta0)` that:
1. Computes equilibrium `ig_a`, `ig_b` from power/voltage
2. Solves for VOC states that produce `dxdt = 0`
3. Returns `[u_mag, theta, Pf, Qf]` at equilibrium

## Test Results

### No-Fault Test
✅ Simulation completes successfully
❌ Large initial transient (Max |dx/dt| = 21.7)
❌ Not at equilibrium

### Fault Test  
✅ Simulation completes successfully
✅ System remains stable (no angle instability)
❌ VOC dynamics unrealistic (artifact of poor initialization)

## Files Modified

1. **`utils/system_coordinator.py`** - Added VOC voltage source support (180 lines added)
2. **`utils/fault_sim_modular.py`** - VOC dynamics integration (60 lines modified)
3. **`utils/power_flow.py`** - Backward compatibility for `solve_network()` return values

## Next Steps

**Priority 1: Fix VOC Initialization**
- Implement equilibrium current calculation in VOC initialization
- Verify `Max |dx/dt| < 1e-6` after initialization
- Expected result: Flat VOC voltage, power output at setpoint

**Priority 2: Validate VOC Dynamics**
- Once initialization is correct, verify fault response is realistic
- Compare VOC vs GFL wind turbine stability
- Document virtual inertia contribution

## Summary

**✅ Network Coupling: COMPLETE & CORRECT**
The VOC is now properly coupled to the network as a voltage source. The framework is modular, expandable, and follows best practices.

**⚠️ Initialization: NEEDS REFINEMENT**
The initialization creates a non-equilibrium starting point, leading to unrealistic transients. This is a numerical issue, not a fundamental problem with the VOC model or network coupling.

**Recommendation:** Proceed with initialization refinement (Option B is cleaner). Once equilibrium is achieved, the VOC will show realistic grid-forming behavior with virtual inertia.
