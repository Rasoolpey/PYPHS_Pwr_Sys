"""Quick test for time-domain impedance scanner (non-interactive)"""
import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from utils.impedance_scanner_td import ImpedanceScannerTD

def main():
    print("="*60)
    print("QUICK TEST: TIME-DOMAIN IMPEDANCE SCANNER")
    print("Testing trajectory difference method")
    print("="*60)

    scanner = ImpedanceScannerTD('test_cases/Kundur_System/kundur_full.json')

    # Test on bus 0 (Gen 1)
    bus_idx = 0
    buses = scanner.list_buses()
    target_bus = buses[bus_idx]
    print(f"\nTarget: {target_bus['name']}")

    # Shorter duration for quick test
    f_max = 50.0
    duration = 30.0  # 30 seconds for quick test
    amplitude = 0.01

    print(f"\nParameters: f_max={f_max} Hz, duration={duration}s, amplitude={amplitude} pu")

    # Run scan (now runs baseline + injection)
    sol, bus_idx = scanner.run_scan(bus_idx=bus_idx, f_max=f_max, amplitude=amplitude, duration=duration)

    # Compute impedances
    freqs, Z_total, Z_network, Z_gen = scanner.compute_all_impedances(sol, bus_idx)

    # Create comparison plot
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    # Magnitude - all three
    ax1.loglog(freqs, np.abs(Z_total), 'b-', linewidth=2, label='Z_total (measured)')
    ax1.loglog(freqs, np.abs(Z_network), 'g--', linewidth=2, label='Z_network (analytical)')
    ax1.loglog(freqs, np.abs(Z_gen), 'r:', linewidth=2, label='Z_gen (analytical)')

    # Show expected parallel impedance
    Z_parallel = (Z_network * Z_gen) / (Z_network + Z_gen)
    ax1.loglog(freqs, np.abs(Z_parallel), 'k--', linewidth=1, alpha=0.5, label='Z_net || Z_gen (expected)')

    ax1.set_ylabel('Impedance Magnitude |Z| (pu)')
    ax1.set_title(f'Impedance Comparison at {target_bus["name"]} (Trajectory Difference Method)')
    ax1.grid(True, which="both", alpha=0.4)
    ax1.set_xlim(0.1, f_max)
    ax1.legend()

    # Phase - all three
    ax2.semilogx(freqs, np.degrees(np.angle(Z_total)), 'b-', linewidth=2, label='Z_total')
    ax2.semilogx(freqs, np.degrees(np.angle(Z_network)), 'g--', linewidth=2, label='Z_network')
    ax2.semilogx(freqs, np.degrees(np.angle(Z_gen)), 'r:', linewidth=2, label='Z_gen')
    ax2.set_ylabel('Phase (degrees)')
    ax2.set_xlabel('Frequency (Hz)')
    ax2.grid(True, which="both", alpha=0.4)
    ax2.set_xlim(0.1, f_max)
    ax2.set_ylim(-180, 180)
    ax2.legend()

    output_dir = 'outputs'
    os.makedirs(output_dir, exist_ok=True)
    output_path = f'{output_dir}/td_impedance_test_trajectory.png'
    plt.savefig(output_path, dpi=150)
    print(f"\nPlot saved to: {output_path}")
    plt.show()

    # Print some diagnostic values
    print("\n" + "="*60)
    print("IMPEDANCE VALUES AT KEY FREQUENCIES:")
    print("="*60)
    for f_target in [0.5, 1.0, 5.0, 10.0, 20.0]:
        idx = np.argmin(np.abs(freqs - f_target))
        print(f"  f={freqs[idx]:.1f} Hz: |Z_total|={np.abs(Z_total[idx]):.4f}, |Z_gen|={np.abs(Z_gen[idx]):.4f}, |Z_net||Z_gen|={np.abs(Z_parallel[idx]):.4f}")

if __name__ == "__main__":
    main()