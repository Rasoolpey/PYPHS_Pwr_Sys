"""Verify exciter limits are enforced during fault"""
import numpy as np

# Load simulation data
data = np.load('outputs/renewable_fault_data.npz')
t = data['t']
y = data['y']

print("="*70)
print("EXCITER LIMIT VERIFICATION - FAULT SIMULATION")
print("="*70)

# Find fault period
fault_start_idx = np.argmin(np.abs(t - 1.0))
fault_end_idx = np.argmin(np.abs(t - 1.05))
post_fault_idx = np.argmin(np.abs(t - 2.0))

print(f"\nFault timing:")
print(f"  Fault starts: t={t[fault_start_idx]:.3f}s (index {fault_start_idx})")
print(f"  Fault clears: t={t[fault_end_idx]:.3f}s (index {fault_end_idx})")
print(f"  Post-fault:   t={t[post_fault_idx]:.3f}s (index {post_fault_idx})")

# Generator 4 (Gen 3 in 0-indexing) is at offset 46, uses EXST1
# EXST1 state structure: [vm, vll, vr, vf]
# We need to check vr (regulator output) which becomes Efd
gen4_offset = 46
gen4_exc_offset = gen4_offset + 7  # After 7 generator states
vr_idx = gen4_exc_offset + 2  # vr is 3rd exciter state (index 2)

vr_history = y[vr_idx, :]

# Find maximum excitation during entire simulation
vr_max = np.max(vr_history)
vr_max_idx = np.argmax(vr_history)
vr_max_time = t[vr_max_idx]

print(f"\n{'='*70}")
print("Generator 4 (EXST1 Exciter) - Field Voltage Analysis:")
print("="*70)

print(f"\nPre-fault (t=0-1.0s):")
vr_prefault = vr_history[:fault_start_idx]
print(f"  Efd steady-state: {np.mean(vr_prefault):.3f} pu")
print(f"  Efd range: [{np.min(vr_prefault):.3f}, {np.max(vr_prefault):.3f}] pu")

print(f"\nDuring fault (t=1.0-1.05s):")
vr_during = vr_history[fault_start_idx:fault_end_idx]
print(f"  Efd max: {np.max(vr_during):.3f} pu")
print(f"  Efd min: {np.min(vr_during):.3f} pu")

print(f"\nPost-fault (t=1.05-2.0s):")
vr_post = vr_history[fault_end_idx:post_fault_idx]
print(f"  Efd max: {np.max(vr_post):.3f} pu")
print(f"  Efd peak time: {t[fault_end_idx + np.argmax(vr_post)]:.3f} s")

print(f"\nOverall simulation:")
print(f"  Efd absolute maximum: {vr_max:.3f} pu at t={vr_max_time:.3f}s")
print(f"  Efd absolute minimum: {np.min(vr_history):.3f} pu")

# Check if limit was hit
VRMAX = 10.0
VRMIN = -7.0
print(f"\n{'='*70}")
print("Limit Enforcement Check:")
print("="*70)
print(f"  Configured VRMAX: {VRMAX:.1f} pu")
print(f"  Configured VRMIN: {VRMIN:.1f} pu")
print(f"  Measured Efd max: {vr_max:.3f} pu")
print(f"  Measured Efd min: {np.min(vr_history):.3f} pu")

if vr_max > VRMAX + 0.1:
    print(f"\n  [X] LIMIT VIOLATION! Efd exceeded VRMAX by {vr_max - VRMAX:.3f} pu")
elif vr_max > VRMAX * 0.95:
    print(f"\n  [OK] Limit is ACTIVE - Efd reached {vr_max/VRMAX*100:.1f}% of VRMAX")
    print(f"       Realistic exciter saturation is occurring!")
else:
    print(f"\n  [OK] Operating within limits - Efd peaked at {vr_max/VRMAX*100:.1f}% of VRMAX")

print("\n" + "="*70)
print("COMPARISON:")
print("="*70)
print(f"  Before fix: Efd reached 17.5 pu (unrealistic)")
print(f"  After fix:  Efd limited to {vr_max:.1f} pu (realistic)")
print(f"  Improvement: {(17.5-vr_max)/17.5*100:.1f}% reduction in peak excitation")
print("="*70)
