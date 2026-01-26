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
    f_max = 300.0  # Max frequency for scan
    duration = 120.0  # Duration of injection
    amplitude = 0.1  # Injection amplitude (pu)
    # Run Scan at Bus 2
    # 60 seconds duration gives good averaging for Welch's method
    sol, bus_idx = scanner.run_scan(bus_idx=2, f_max=f_max, amplitude=0.01, duration=duration)
    
    # Process
    freqs, Z_est = scanner.post_process_tfe(sol, bus_idx)
    
    # Plot detailed system response
    print("\n" + "="*60)
    print("PLOTTING SYSTEM RESPONSE")
    print("="*60)
    scanner.plot_system_response('outputs/system_response.png')
    
    # Plotting impedance
    output_dir = 'outputs'
    os.makedirs(output_dir, exist_ok=True)
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    
    # Magnitude
    ax1.loglog(freqs, np.abs(Z_est), 'b-', linewidth=2)
    ax1.set_ylabel('Impedance Magnitude |Z| (pu)')
    ax1.set_title(f'Measured Impedance at Bus {scanner.list_buses()[bus_idx]["bus_id"]}')
    ax1.grid(True, which="both", alpha=0.4)
    ax1.set_xlim(0.1, f_max)
    
    # Phase
    ax2.semilogx(freqs, np.degrees(np.angle(Z_est)), 'r-', linewidth=2)
    ax2.set_ylabel('Phase (degrees)')
    ax2.set_xlabel('Frequency (Hz)')
    ax2.grid(True, which="both", alpha=0.4)
    ax2.set_xlim(0.1, f_max)
    ax2.set_ylim(-180, 180)
    
    output_path = f'{output_dir}/td_impedance_result_smooth.png'
    plt.savefig(output_path, dpi=150)
    print(f"Plot saved to: {output_path}")
    plt.show()

if __name__ == "__main__":
    main()