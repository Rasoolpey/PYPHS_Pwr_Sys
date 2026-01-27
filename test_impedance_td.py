"""
Test Time-Domain Impedance Scanning (White Noise Method)
"""
import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from utils.impedance_scanner_td import ImpedanceScannerTD

def main():
    print("="*60)
    print("SYSTEM ID: IMPEDANCE SCANNER (WHITE NOISE)")
    print("="*60)
    
    scanner = ImpedanceScannerTD('test_cases/Kundur_System/kundur_full.json')
    
    # Show available buses
    print("\nScanning system topology...")
    buses = scanner.list_buses()
    
    print("\nAvailable Buses for Impedance Scanning:")
    print(f"{'Index':<8} | {'Bus ID':<8} | {'Name'}")
    print("-" * 40)
    for b in buses:
        print(f"{b['idx']:<8} | {b['bus_id']:<8} | {b['name']}")
    print("-" * 40)
    
    # User Interaction
    while True:
        try:
            selection = input("\nEnter Bus Index to scan (0-3) or 'q' to quit: ")
            if selection.lower() == 'q':
                sys.exit()
            bus_idx = int(selection)
            if 0 <= bus_idx < len(buses):
                break
            print("Invalid index. Please try again.")
        except ValueError:
            print("Please enter a valid number.")

    target_bus = buses[bus_idx]
    print(f"\nSelected: {target_bus['name']}")
    
    f_max = 50.0  # Max frequency for scan
    duration = 60.0  # Duration of injection
    amplitude = 0.01  # Injection amplitude (pu) - Reduced to ensure small-signal linearity
    
    # Run Scan at selected bus
    sol, bus_idx = scanner.run_scan(bus_idx=bus_idx, f_max=f_max, amplitude=amplitude, duration=duration)

    # Plot detailed system response
    print("\n" + "="*60)
    print("PLOTTING SYSTEM RESPONSE")
    print("="*60)
    output_dir = 'outputs'
    os.makedirs(output_dir, exist_ok=True)
    scanner.plot_system_response('outputs/system_response.png')

    # Compute ALL THREE impedances
    freqs, Z_total, Z_network, Z_gen = scanner.compute_all_impedances(sol, bus_idx)

    # Plotting all impedances in combined plot

    combined_output = f'{output_dir}/impedance_analysis_bus_{target_bus["bus_id"]}.png'
    scanner.plot_all_impedances(freqs, Z_total, Z_network, Z_gen, target_bus["name"], combined_output)

    # Also create comparison plot with all three overlaid
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    # Magnitude - all three
    ax1.loglog(freqs, np.abs(Z_total), 'b-', linewidth=2, label='Z_total (measured)')
    ax1.loglog(freqs, np.abs(Z_network), 'g--', linewidth=2, label='Z_network (analytical)')
    ax1.loglog(freqs, np.abs(Z_gen), 'r:', linewidth=2, label='Z_gen (analytical)')
    ax1.set_ylabel('Impedance Magnitude |Z| (pu)')
    ax1.set_title(f'Impedance Comparison at {target_bus["name"]}')
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

    output_path = f'{output_dir}/td_impedance_bus_{target_bus["bus_id"]}.png'
    plt.savefig(output_path, dpi=150)
    print(f"Comparison plot saved to: {output_path}")
    plt.show()

if __name__ == "__main__":
    main()