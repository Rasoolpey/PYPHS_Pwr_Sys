# VOC (Virtual Oscillator Control) Implementation - Issues and Solutions

**Last Updated**: February 15, 2026  
**Status**: Implementation Complete, Stability Issues Under Investigation

---

## 🔴 Critical Issues

### Issue 1: Large Power Oscillations in VOC Operation

**Symptom**:
- VOC active power oscillates wildly: **-0.35 to +0.35 pu** (target: 0.35 pu constant)
- Reactive power swings: **-0.35 to +0.30 pu** (target: 0.074 pu)
- Voltage magnitude oscillates: **0.75 to 1.25 pu** (target: 1.03 pu)
- Oscillation period: ~6-7 seconds (inter-area mode frequency)
- **System generators remain stable** (±0.0004% frequency deviation)

**Observed in**:
- Commit `7682c33`: "GFM has been integrated properly and works fantastically" 
- Commit `d7a51b2`: "voc implemented and readme updated but voc init doesn't work properly"
- Both commits show identical oscillation patterns

**What We Know**:
- LPF tuning (ω_lpf = 1-63 rad/s) does **NOT** fix the issue
- All tested LPF frequencies produce **identical** oscillation patterns
- This suggests the problem is **NOT** in the measurement filter
- The oscillation appears to be a **fundamental control loop instability**

**Probable Solutions** (guesses, not certain):

1. **Hypothesis: Current Direction Sign Error**
   - **Reasoning**: The VOC might be seeing grid current with wrong polarity
   - In network solver: `I_voc = (u_voc - V_bus) * y_filt` (current FROM VOC TO grid)
   - In VOC dynamics: `P_inst = u_a * ig_a + u_b * ig_b` (assumes current INTO VOC for +P)
   - **Possible fix**: Negate the current passed to VOC: `ig_a = -np.real(I_voc)`, `ig_b = -np.imag(I_voc)`
   - **Confidence**: 60% - sign errors are common in power flow conventions

2. **Hypothesis: Reference Frame Mismatch**
   - **Reasoning**: Generators use COI-referenced angles, but VOC voltage angle might not be properly synchronized
   - We added COI tracking (`theta_coi`) but the **initial angle offset** might be wrong
   - At t=0, VOC `theta_init` from power flow is in absolute frame, but generators start at `delta=0` (COI frame)
   - **Possible fix**: Initialize VOC theta relative to COI: `theta_init = V_bus_angle - 0.0` (where 0.0 is COI reference angle at t=0)
   - **Confidence**: 40% - angle initialization is complex and error-prone

3. **Hypothesis: P-f Droop Gain Too High**
   - **Reasoning**: Current setting `mp = 0.05` (5% droop) might be too aggressive for this system
   - With oscillating power, P-f droop creates large frequency swings → more power oscillation (positive feedback)
   - Standard synchronous generators have droop ~3-5%, but VOC might need different tuning in weak grids
   - **Possible fix**: Reduce `mp` to 0.01-0.02 (1-2% droop)
   - **Confidence**: 30% - but system stability shouldn't depend this critically on droop gain

4. **Hypothesis: Q-V Control Instability (Pump-Damp Switching)**
   - **Reasoning**: The VOC uses a sign-switching Q-V control (`xi2` sign changes based on passivity condition)
   - Rapid switching between pump/damp modes could create limit cycles
   - The `hysteresis` parameter (0.01) might be too small to prevent chattering
   - **Possible fix**: 
     - Increase hysteresis to 0.05-0.1
     - OR disable Q-V control temporarily (set `xi2 = 0`) to isolate P-f droop
   - **Confidence**: 50% - pump-damp logic is known to be sensitive

5. **Hypothesis: Filter Drop Compensation Error**
   - **Reasoning**: VOC initialization calculates `u_voc = V_bus + I_inj * Z_filt` to compensate filter voltage drop
   - If `I_inj` is calculated with **wrong sign**, the compensation goes in wrong direction
   - This would create voltage error → reactive power error → oscillation
   - **Possible fix**: Check sign of `I_inj` used in filter drop calculation in `power_flow.py`
   - **Confidence**: 45% - voltage/current direction is a common source of errors

---

### Issue 2: VOC Initialization Transients

**Symptom**:
- At t=0: `dPf/dt = -9.9 to -21.7` (huge derivative for power filter state)
- At t=0: `dQf/dt = -4.4` (large derivative for reactive power filter)
- Initial transient lasts ~50-100ms before settling
- Voltage magnitude `du/dt = 0.039` is acceptable

**What We Know**:
- Power flow sets `Pf_init = Pref` and `Qf_init = Qref` (filter states = targets)
- But instantaneous power `P_inst` at t=0 is **NOT** equal to `Pref`
- This creates large initial `dPf/dt = ω_lpf * (P_inst - Pref)`

**Probable Solutions** (guesses):

1. **Hypothesis: Instantaneous Power Mismatch at t=0**
   - **Reasoning**: The VOC calculates `P_inst = u_a * ig_a + u_b * ig_b` from grid current
   - At t=0, the grid current from network solver might not match the power flow target
   - Need to ensure `P_inst(t=0) ≈ Pref` by adjusting initial current injection
   - **Possible fix**: In power_flow.py VOC initialization, iterate until `P_inst` converges to `Pref` (within 1e-4 pu)
   - **Confidence**: 70% - this is likely the root cause of initialization transients

2. **Hypothesis: Network Solve Not Including VOC at t=0**
   - **Reasoning**: If initial network solve doesn't include VOC voltage injection, the terminal voltage will jump when VOC activates
   - Need to ensure network is solved **with VOC voltages** during initialization
   - **Possible fix**: Verify `voc_voltages` is passed to `coordinator.solve_network()` in initialization refinement loop
   - **Confidence**: 50% - this should already be done, but worth double-checking

---

## 🟡 Medium Priority Issues

### Issue 3: VOC Angle Plotting Confusion

**Symptom**:
- VOC output angle drifts to **850° after 15 seconds**
- User initially thought this was an error

**Clarification**:
- This is **CORRECT** behavior! The angle is rotating at 60 Hz (1.0 rad/s in pu time)
- After 15s: `θ = 1.0 rad/s × 15s = 15 rad = 859°` ✓
- This is the **absolute** phase angle of the voltage phasor

**Probable Solutions**:

1. **Improve Plot Labeling**
   - **Reasoning**: The plot should clarify this is "absolute phase angle" not "angle deviation"
   - Add note to plot: "Absolute phase (rotates at 60 Hz)"
   - **Confidence**: 100% - this is just a documentation issue

2. **Plot Relative Angle Instead**
   - **Reasoning**: Plot `theta - omega0 * t` to show angle **deviation** from nominal
   - This would show a flat line at 0° in steady-state
   - **Possible fix**: Add second subplot showing `delta_theta = theta - (omega0 * t + theta_init)`
   - **Confidence**: 80% - this would be more intuitive for users

---

### Issue 4: Unclear VOC State Count

**Symptom**:
- Test output shows **68 states total** (should be 67 for IEEE 14-bus + 1 VOC)
- Not clear if this is a counting error or extra state somewhere

**What We Know**:
- Generators: 4 × 7 = 28 states
- Exciters: 5+5+5+5+4 = 24 states
- Governors: 2+2+6+6 = 16 states
- VOC: 1 × 4 = 4 states
- **Total**: 28 + 24 + 16 + 4 = **72 states?**

**Possible Issue**:
- Exciter state count might be wrong (5+5+5+5+4 = 24, but listed as "Exc=[5, 5, 5, 5, 4]")
- Governor state count: "Gov=[2, 2, 6, 6]" = 16 states ✓
- Generator state count: "Gen=[7, 7, 7, 7]" = 28 states ✓

**Probable Solution**:
- Recount exciter states per generator in `fault_sim_modular.py`
- Verify `exc_state_counts` array is correctly built
- **Confidence**: 90% - this is likely just a display formatting issue

---

## 🟢 Low Priority / Investigation Needed

### Issue 5: COI Tracking Implementation

**What Was Done**:
- Added `self.theta_coi` to track integrated COI angle
- Added `self.t_prev` to track time for integration
- Transform VOC angle: `theta_net = theta - self.theta_coi` before network injection

**Possible Concerns**:
- COI angle initialization: `self.theta_coi = 0.0` at t=0 - is this correct?
- COI angle integration: `self.theta_coi += omega_coi * dt` - should this be `omega0 * dt` instead?
- **Confidence**: 60% that current implementation is correct

**Investigation Needed**:
- Compare generator `delta` evolution with VOC `theta` evolution
- Check if `theta_coi` grows linearly (expected: ~377 rad/s in absolute time, 1 rad/s in pu time)

---

### Issue 6: VOC Filter Impedance

**Current Implementation**:
- `Z_filt = Rf + 1j * omega0 * Lf` where `omega0 = 1.0` (pu)
- For `Rf = 0.01, Lf = 0.08`: `Z_filt = 0.01 + 0.08j` → `|Z| = 0.0806 pu`

**Possible Concern**:
- In physical systems, `ω = 2π × 60 rad/s`
- In per-unit time, `ω = 1.0 rad/s`
- Filter impedance: `Z = R + jωL` should use **base frequency** for conversion
- Current code uses `omega0 = 1.0` which is correct for pu system ✓

**Conclusion**: Filter impedance is **correctly** implemented
- No action needed
- **Confidence**: 95%

---

## 📋 Next Steps (Recommended Investigation Order)

### Step 1: Test Current Direction Hypothesis (Highest Priority)
**Action**: Try negating VOC grid current in `fault_sim_modular.py`:
```python
# In VOC dynamics section:
ig_a = -np.real(I_grid_complex)  # Add negative sign
ig_b = -np.imag(I_grid_complex)  # Add negative sign
```
**Expected Result**: If correct, oscillations should disappear or significantly reduce

---

### Step 2: Isolate P-f Droop vs Q-V Control
**Action**: Temporarily disable Q-V control in VOC JSON:
```json
"VOC_INVERTER": [{
  "xi2": 0.0,  // Disable Q-V control (was 0.42)
  "mp": 0.05   // Keep P-f droop active
}]
```
**Expected Result**: If Q-V control is the issue, oscillations should stop (but Q might drift)

---

### Step 3: Verify Instantaneous Power at t=0
**Action**: Add diagnostic print in `power_flow.py` VOC initialization:
```python
print(f"  VOC instantaneous power: P_inst={P_inst:.4f} pu (target={Pref:.4f})")
print(f"  Filter states: Pf={Pf_init:.4f}, Qf={Qf_init:.4f}")
print(f"  Initial derivatives: dPf/dt={dPf_dt:.4f}, dQf/dt={dQf_dt:.4f}")
```
**Expected Result**: Large mismatch between `P_inst` and `Pref` would confirm initialization issue

---

### Step 4: Compare With and Without COI Tracking
**Action**: Temporarily disable COI transformation:
```python
# In fault_sim_modular.py, line ~599:
theta_net = theta  # - self.theta_coi  # Disable COI transform
```
**Expected Result**: If COI is causing issues, behavior should change significantly

---

### Step 5: Reduce P-f Droop Gain
**Action**: Try lower droop gain in VOC JSON:
```json
"VOC_INVERTER": [{
  "mp": 0.01  // Reduce from 0.05 to 0.01 (1% droop)
}]
```
**Expected Result**: If gain is too high, oscillations should reduce in amplitude

---

## 📝 Notes for Future Investigation

### Theoretical Background (May Be Relevant)

1. **VOC Stability Conditions (from literature)**:
   - Stability requires: `mp > 0` (P-f droop positive)
   - Passivity requires: `sign(xi2) = -sign(Q_ratio × Phi)`
   - Small Cf can cause high-frequency oscillations
   - Large mp can cause low-frequency oscillations (inter-area modes)

2. **Common VOC Implementation Errors**:
   - Sign errors in current measurement
   - Reference frame mismatches (abc vs αβ vs dq)
   - Filter parameter scaling (SI vs pu)
   - Saturation/limiting not properly handled

3. **Grid-Following vs Grid-Forming**:
   - GFL (current source): Injects current setpoints, grid determines voltage
   - GFM (voltage source): Sets voltage, grid determines current
   - VOC is GFM → Should behave like synchronous generator
   - Current implementation might have GFL characteristics accidentally

---

## 🎯 Success Criteria

VOC implementation is considered **successful** when:
- [ ] Active power steady-state error < 0.01 pu (current: oscillating ±0.35 pu)
- [ ] Reactive power steady-state error < 0.01 pu (current: oscillating ±0.30 pu)
- [ ] Voltage magnitude steady-state error < 0.01 pu (current: oscillating ±0.15 pu)
- [ ] Initial transient settles within 100ms (current: ~50ms OK ✓)
- [ ] Frequency deviation < 0.001 pu (current: OK for generators, VOC unmeasured)
- [ ] System remains stable during 3-phase fault (not yet tested)

---

## 📚 References to Check

Papers/Standards that might help:
- Jouini, T., et al. (2019). "Grid-Friendly Matching of Synchronous Machines by Tapping into the DC Link"
- IEEE Std 1547-2018: Interconnection requirements for distributed resources
- WECC Generic Models for Renewable Energy (REGC_A, REEC_A) - grid-following comparison

Code to review:
- Original VOC paper implementation (if available)
- Similar GFM implementations in other tools (PSCAD, PSSE, PowerFactory)

---

**END OF TODO**
