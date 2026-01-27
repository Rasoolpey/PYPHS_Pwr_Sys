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

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    # Compute positive-sequence impedance
    Z_dd = Z_dq[:, 0, 0]
    Z_qq = Z_dq[:, 1, 1]
    Z_dq_cross = Z_dq[:, 0, 1]  # d to q coupling
    Z_qd_cross = Z_dq[:, 1, 0]  # q to d coupling (main reactive term)
    Z_pos = (Z_dd + Z_qq)/2 + 1j*(Z_qd_cross - Z_dq_cross)/2

    # Plot 1: Self-impedances (diagonal terms)
    ax1 = axes[0, 0]
    ax1.loglog(f, np.abs(Z_dd), 'b-', label='|Z_dd| (d-axis self)', linewidth=2)
    ax1.loglog(f, np.abs(Z_qq), 'r--', label='|Z_qq| (q-axis self)', linewidth=2)
    ax1.set_ylabel('Magnitude (pu)')
    ax1.set_title('Self-Impedances (Diagonal)')
    ax1.grid(True, which="both", ls="-", alpha=0.4)
    ax1.legend()

    # Plot 2: Cross-coupling impedances (off-diagonal terms)
    ax2 = axes[0, 1]
    ax2.loglog(f, np.abs(Z_qd_cross), 'g-', label='|Z_qd| (d->q, ~Xd)', linewidth=2)
    ax2.loglog(f, np.abs(Z_dq_cross), 'm--', label='|Z_dq| (q->d, ~Xq)', linewidth=2)
    ax2.set_ylabel('Magnitude (pu)')
    ax2.set_title('Cross-Coupling Impedances (Off-Diagonal)')
    ax2.grid(True, which="both", ls="-", alpha=0.4)
    ax2.legend()

    # Plot 3: Positive-sequence impedance magnitude
    ax3 = axes[1, 0]
    ax3.loglog(f, np.abs(Z_pos), 'k-', label='|Z_pos|', linewidth=2)
    ax3.loglog(f, Z_mag, 'b--', label='|Z| (from scanner)', linewidth=1.5, alpha=0.7)
    ax3.set_ylabel('Magnitude (pu)')
    ax3.set_xlabel('Frequency (Hz)')
    ax3.set_title('Positive-Sequence Impedance')
    ax3.grid(True, which="both", ls="-", alpha=0.4)
    ax3.legend()

    # Plot 4: Positive-sequence phase
    ax4 = axes[1, 1]
    ax4.semilogx(f, np.degrees(np.angle(Z_pos)), 'k-', label='Phase(Z_pos)', linewidth=2)
    ax4.semilogx(f, np.degrees(np.angle(Z_qd_cross)), 'g--', label='Phase(Z_qd)', linewidth=1.5, alpha=0.7)
    ax4.set_ylabel('Phase (degrees)')
    ax4.set_xlabel('Frequency (Hz)')
    ax4.set_title('Phase Response')
    ax4.grid(True, which="both", ls="-", alpha=0.4)
    ax4.legend()

    fig.suptitle(f'DQ Impedance Analysis at {target_bus["name"]}', fontsize=14)
    plt.tight_layout()
    
    # Create outputs directory if it doesn't exist
    import os
    os.makedirs('outputs', exist_ok=True)
    
    output_file = os.path.join('outputs', f'impedance_scan_bus_{target_bus["bus_id"]}.png')
    plt.savefig(output_file)
    print(f"Plot saved to {output_file}")
    plt.show()

if __name__ == "__main__":
    main()