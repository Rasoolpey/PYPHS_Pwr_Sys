"""Analyze stability of fault simulation"""
import numpy as np

# Load simulation data
data = np.load('outputs/renewable_fault_data.npz')
t = data['t']
y = data['y']

print("="*70)
print("FAULT SIMULATION STABILITY ANALYSIS")
print("="*70)

print(f"\nSimulation Results:")
print(f"  Final time: {t[-1]:.3f} s (target was 15.0 s)")
print(f"  Total time points: {len(t)}")

if t[-1] < 14.0:
    print(f"\n  [!] SIMULATION STOPPED EARLY - System likely unstable!")
else:
    print(f"\n  [OK] Simulation completed successfully")

print(f"\nChecking for instability indicators:")

# Check rotor angles (first 4 generators: offsets 0, 14, 28, 46)
state_offsets = [0, 14, 28, 46]
M_vals = [8.0, 13.0, 10.0, 10.0]

print(f"\n  Rotor Angle Analysis:")
for i, offset in enumerate(state_offsets):
    delta = np.degrees(y[offset, :])
    delta_min = np.min(delta)
    delta_max = np.max(delta)
    delta_swing = delta_max - delta_min
    
    print(f"    Gen {i+1}: swing = {delta_swing:.1f} deg, "
          f"range = [{delta_min:.1f}, {delta_max:.1f}] deg")
    
    if delta_swing > 180:
        print(f"           => POLE SLIP! Generator lost synchronism")
    elif delta_swing > 120:
        print(f"           => SEVERE oscillations - near instability limit")

# Check omega deviation
print(f"\n  Frequency Analysis:")
for i, offset in enumerate(state_offsets):
    p = y[offset+1, :]
    omega = p / M_vals[i]
    omega_dev_max = np.max(np.abs(omega - 1.0))
    
    print(f"    Gen {i+1}: max freq deviation = {omega_dev_max*60:.3f} Hz")
    
    if omega_dev_max > 0.05:
        print(f"           => CRITICAL - exceeds typical relay limits (~3 Hz)")

# Check if simulation stopped due to instability
if t[-1] < 5.0:
    print(f"\n{'='*70}")
    print("CONCLUSION: SYSTEM LOST STABILITY")
    print("="*70)
    print("\nThe system with wind turbine integration CANNOT withstand this fault.")
    print("\nReasons:")
    print("  1. Reduced system inertia (41 vs 51 MWs/MVA)")
    print("  2. Lower damping ratio (19.6% inertia reduction)")
    print("  3. Severe fault (0.01j at Bus 9)")
    print("  4. Wind turbine provides no transient support (GFL control)")
    print("\nPossible solutions:")
    print("  - Reduce fault severity (increase impedance)")
    print("  - Reduce fault duration (0.1s -> 0.05s)")
    print("  - Add Power System Stabilizers (PSS)")
    print("  - Implement Grid-Forming (GFM) control with virtual inertia")
    print("  - Increase damping coefficient D on remaining generators")
else:
    print(f"\n{'='*70}")
    print("CONCLUSION: SYSTEM MAINTAINED STABILITY")
    print("="*70)

print("="*70)
