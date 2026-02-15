#!/usr/bin/env python3
"""
Test VOC with different LPF settings to find optimal damping.

Tests omega_lpf values from 1 to 60 rad/s to investigate power oscillations.
"""

import numpy as np
import matplotlib.pyplot as plt
from utils.fault_sim_modular import ModularFaultSimulator
import json
import tempfile
import os

def run_voc_test(omega_lpf, t_end=15.0):
    """Run VOC simulation with specified LPF cutoff frequency."""
    
    # Load system with VOC
    system_json = 'test_cases/ieee14bus/renewable_resources_adoption/ieee14_voc_system.json'
    sim_json = 'test_cases/ieee14bus/renewable_resources_adoption/simulation_fault.json'
    
    # Load and modify system JSON with new omega_lpf
    with open(system_json, 'r') as f:
        system_data = json.load(f)
    
    # Override VOC omega_lpf parameter
    if 'VOC_INVERTER' in system_data and len(system_data['VOC_INVERTER']) > 0:
        system_data['VOC_INVERTER'][0]['omega_lpf'] = omega_lpf
        print(f"\nTesting with omega_lpf = {omega_lpf:.2f} rad/s (tau = {1000/omega_lpf:.1f} ms)")
    
    # Write modified system to temporary file
    with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
        json.dump(system_data, f, indent=2)
        temp_system_json = f.name
    
    try:
        sim = ModularFaultSimulator(system_json=temp_system_json, simulation_json=sim_json)
    
        # Initialize
        x0 = sim.initialize_equilibrium(run_power_flow=True)
        
        # Disable fault
        sim.disable_fault()
        
        # Simulate (no fault)
        print(f"  Simulating {t_end}s...")
        sol = sim.simulate(x0, t_end=t_end, n_points=3000)
        
        if sol is None:
            return None
        
        # Extract time and states
        t = sol.t
        x_hist = sol.y.T  # (n_time, n_states)
    finally:
        # Clean up temp file
        if os.path.exists(temp_system_json):
            os.remove(temp_system_json)
    
    # Extract VOC states
    if sim.n_voc > 0:
        voc_offset = sim.voc_offsets[0]
        start = voc_offset['start']
        u_mag = x_hist[:, start + 0]
        theta = x_hist[:, start + 1]
        P_voc = x_hist[:, start + 2]
        Q_voc = x_hist[:, start + 3]
        
        # Compute metrics (after 10% settling)
        settle_idx = int(len(P_voc) * 0.1)
        P_mean = np.mean(P_voc[settle_idx:])
        P_std = np.std(P_voc[settle_idx:])
        P_peak = np.max(np.abs(P_voc[settle_idx:] - P_mean))
        
        Q_mean = np.mean(Q_voc[settle_idx:])
        Q_std = np.std(Q_voc[settle_idx:])
        
        u_mean = np.mean(u_mag[settle_idx:])
        u_std = np.std(u_mag[settle_idx:])
        
        return {
            't': t,
            'P': P_voc,
            'Q': Q_voc,
            'u_mag': u_mag,
            'P_mean': P_mean,
            'P_std': P_std,
            'P_peak': P_peak,
            'Q_mean': Q_mean,
            'Q_std': Q_std,
            'u_mean': u_mean,
            'u_std': u_std,
            'omega_lpf': omega_lpf
        }
    else:
        return None


def main():
    print("="*80)
    print("  VOC LPF TUNING - INVESTIGATING POWER OSCILLATIONS")
    print("="*80)
    
    # Test different omega_lpf values
    # Current: 62.83 rad/s (10 Hz)
    # Target range: 1-20 rad/s (0.16-3.2 Hz)
    omega_lpf_values = [1.0, 3.14, 6.28, 12.57, 20.0, 31.42, 62.83]
    
    results = []
    
    for omega_lpf in omega_lpf_values:
        result = run_voc_test(omega_lpf, t_end=15.0)
        if result is not None:
            results.append(result)
            print(f"    P: mean={result['P_mean']:.3f} pu, std={result['P_std']:.3f} pu, peak_dev={result['P_peak']:.3f} pu")
            print(f"    Q: mean={result['Q_mean']:.3f} pu, std={result['Q_std']:.3f} pu")
            print(f"    V: mean={result['u_mean']:.3f} pu, std={result['u_std']:.4f} pu")
    
    if len(results) == 0:
        print("No results to plot!")
        return
    
    # Plot comparison
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    
    for result in results:
        label = f"lpf={result['omega_lpf']:.1f} rad/s"
        axes[0].plot(result['t'], result['P'], label=label, alpha=0.7)
        axes[1].plot(result['t'], result['Q'], label=label, alpha=0.7)
        axes[2].plot(result['t'], result['u_mag'], label=label, alpha=0.7)
    
    axes[0].set_ylabel('Active Power (pu)')
    axes[0].axhline(0.35, color='k', linestyle='--', alpha=0.3, label='Pref=0.35')
    axes[0].legend(loc='best', fontsize=8)
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title('VOC Active Power vs LPF Cutoff Frequency')
    
    axes[1].set_ylabel('Reactive Power (pu)')
    axes[1].axhline(0.074, color='k', linestyle='--', alpha=0.3, label='Qref=0.074')
    axes[1].legend(loc='best', fontsize=8)
    axes[1].grid(True, alpha=0.3)
    
    axes[2].set_ylabel('Voltage Magnitude (pu)')
    axes[2].axhline(1.03, color='k', linestyle='--', alpha=0.3, label='Vref=1.03')
    axes[2].set_xlabel('Time (s)')
    axes[2].legend(loc='best', fontsize=8)
    axes[2].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('outputs/voc_lpf_tuning.png', dpi=150)
    print(f"\n  Comparison plot saved: outputs/voc_lpf_tuning.png")
    
    # Summary table
    print("\n" + "="*80)
    print("  SUMMARY TABLE")
    print("="*80)
    print(f"{'omega_lpf (rad/s)':<20} {'tau (ms)':<10} {'P_std (pu)':<12} {'P_peak (pu)':<12} {'u_std (pu)':<12}")
    print("-"*80)
    for result in results:
        tau_ms = 1000.0 / result['omega_lpf']
        print(f"{result['omega_lpf']:<15.2f} {tau_ms:<10.1f} {result['P_std']:<12.4f} {result['P_peak']:<12.4f} {result['u_std']:<12.5f}")
    
    # Find best (minimum std)
    best_idx = np.argmin([r['P_std'] for r in results])
    best = results[best_idx]
    print(f"\n  BEST: omega_lpf = {best['omega_lpf']:.2f} rad/s (tau = {1000/best['omega_lpf']:.1f} ms)")
    print(f"        P_std = {best['P_std']:.4f} pu, P_peak_dev = {best['P_peak']:.4f} pu")
    print("="*80)


if __name__ == '__main__':
    main()
