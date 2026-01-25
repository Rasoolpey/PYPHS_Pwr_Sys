"""
Test Script for Impedance Scanning Module
"""
import sys
import numpy as np
import matplotlib.pyplot as plt
from utils.impedance_scanner import ImpedanceScanner

def main():
    print("="*60)
    print("PORT-HAMILTONIAN SYSTEM - IMPEDANCE SCANNER")
    print("="*60)
    
    # 1. Initialize System
    json_file = 'test_cases/Kundur_System/kundur_full.json'
    scanner = ImpedanceScanner(json_file)
    
    # 2. Show available buses
    print("\nScanning system topology...")
    buses = scanner.list_buses()
    
    print("\nAvailable Buses for Impedance Scanning:")
    print(f"{'Index':<8} | {'Bus ID':<8} | {'Name'}")
    print("-" * 40)
    for b in buses:
        print(f"{b['idx']:<8} | {b['bus_id']:<8} | {b['name']}")
    print("-" * 40)
    
    # 3. User Interaction
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
    
    # 4. Define Frequency Range
    # Logarithmic sweep from 0.1 Hz to 100 Hz
    freqs = np.logspace(-1, 2, 100) 
    
    # 5. Run Scan
    f, Z_mag, Z_phase, Z_dq = scanner.scan_impedance(bus_idx, freqs)
    
    # 6. Plot Results
    print("\nPlotting results...")
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    
    # Magnitude Plot
    # Plotting dd and qq axis impedances
    ax1.loglog(f, np.abs(Z_dq[:, 0, 0]), 'b-', label='Z_dd (d-axis)', linewidth=2)
    ax1.loglog(f, np.abs(Z_dq[:, 1, 1]), 'r--', label='Z_qq (q-axis)', linewidth=2)
    ax1.set_ylabel('Magnitude (Ohm/pu)')
    ax1.set_title(f'Impedance Spectrum at {target_bus["name"]}')
    ax1.grid(True, which="both", ls="-", alpha=0.4)
    ax1.legend()
    
    # Phase Plot
    ax2.semilogx(f, np.degrees(np.angle(Z_dq[:, 0, 0])), 'b-', label='Z_dd', linewidth=2)
    ax2.semilogx(f, np.degrees(np.angle(Z_dq[:, 1, 1])), 'r--', label='Z_qq', linewidth=2)
    ax2.set_ylabel('Phase (degrees)')
    ax2.set_xlabel('Frequency (Hz)')
    ax2.grid(True, which="both", ls="-", alpha=0.4)
    ax2.legend()
    
    # Create outputs directory if it doesn't exist
    import os
    os.makedirs('outputs', exist_ok=True)
    
    output_file = os.path.join('outputs', f'impedance_scan_bus_{target_bus["bus_id"]}.png')
    plt.savefig(output_file)
    print(f"Plot saved to {output_file}")
    plt.show()

if __name__ == "__main__":
    main()