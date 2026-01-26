"""
Test Script for IMTB-style MIMO Impedance Scanning
Updated: Output Table includes Phase.
"""
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from utils.imtb_scanner import IMTBScanner

def main():
    print("="*60)
    print("IMTB MIMO IMPEDANCE SCANNER")
    print("="*60)
    
    scanner = IMTBScanner('test_cases/Kundur_System/kundur_full.json')
    
    # Range
    freqs = np.logspace(-1, 1.7, 30) 
    
    # Run
    f_vec, Z_mimo = scanner.run_mimo_scan(bus_idx=2, freqs_hz=freqs, 
                                          duration=10.0, amplitude=0.02)
    
    # --- PRINT DETAILED RESULTS ---
    print("\n" + "="*80)
    print("IMPEDANCE MATRIX RESULTS (Sample)")
    print("="*80)
    # Header
    print(f"{'Freq':<6} | {'Zdd':<16} | {'Zqq':<16} | {'Zdq':<16} | {'Zqd':<16}")
    print(f"{'Hz':<6} | {'Mag':<7} {'Ang':<8} | {'Mag':<7} {'Ang':<8} | {'Mag':<7} {'Ang':<8} | {'Mag':<7} {'Ang':<8}")
    print("-" * 85)
    
    for i in range(0, len(f_vec), 4): # Print every 4th
        f = f_vec[i]
        
        # Extract Mag and Phase (deg)
        zdd_m = np.abs(Z_mimo[i, 0, 0]); zdd_p = np.degrees(np.angle(Z_mimo[i, 0, 0]))
        zqq_m = np.abs(Z_mimo[i, 1, 1]); zqq_p = np.degrees(np.angle(Z_mimo[i, 1, 1]))
        zdq_m = np.abs(Z_mimo[i, 0, 1]); zdq_p = np.degrees(np.angle(Z_mimo[i, 0, 1]))
        zqd_m = np.abs(Z_mimo[i, 1, 0]); zqd_p = np.degrees(np.angle(Z_mimo[i, 1, 0]))
        
        print(f"{f:<6.2f} | {zdd_m:<7.3f} {zdd_p:<8.1f} | {zqq_m:<7.3f} {zqq_p:<8.1f} | {zdq_m:<7.3f} {zdq_p:<8.1f} | {zqd_m:<7.3f} {zqd_p:<8.1f}")
    print("-" * 85)
    
    # Plotting
    fig, axes = plt.subplots(2, 2, figsize=(12, 10), sharex=True)
    titles = [['Zdd', 'Zdq'], ['Zqd', 'Zqq']]
    
    for i in range(2):
        for j in range(2):
            ax = axes[i, j]
            Z_ij = Z_mimo[:, i, j]
            
            ln1 = ax.loglog(f_vec, np.abs(Z_ij), 'b.-', linewidth=2, label='Mag')
            ax.set_ylabel('|Z| (pu)', color='b')
            ax.tick_params(axis='y', labelcolor='b')
            ax.grid(True, which='both', alpha=0.3)
            ax.set_title(titles[i][j])
            
            ax2 = ax.twinx()
            ln2 = ax2.semilogx(f_vec, np.degrees(np.angle(Z_ij)), 'r--', linewidth=1.5, label='Phase')
            ax2.set_ylabel('Phase (deg)', color='r')
            ax2.tick_params(axis='y', labelcolor='r')
            ax2.set_ylim(-180, 180)
            
            if i == 1:
                ax.set_xlabel('Frequency (Hz)')

    plt.suptitle(f"MIMO Impedance at Bus {scanner.list_buses()[2]['bus_id']}", fontsize=14)
    plt.tight_layout()
    plt.savefig('outputs/imtb_mimo_scan.png')
    print("Plot saved to outputs/imtb_mimo_scan.png")
    plt.show()

if __name__ == "__main__":
    main()