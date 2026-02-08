"""
Test power flow with Kundur 4-machine system.
Demonstrates automatic slack bus selection for multi-machine systems.
"""
import sys
import numpy as np
sys.path.insert(0, '.')
from utils.system_builder import PowerSystemBuilder
from utils.system_coordinator import PowerSystemCoordinator
from utils.power_flow import run_power_flow

print("\n" + "="*70)
print("   KUNDUR 4-MACHINE SYSTEM POWER FLOW TEST")
print("="*70)

try:
    # Build system
    print("\n[Step 1] Building Kundur system from JSON...")
    builder = PowerSystemBuilder('test_cases/Kundur_System/kundur_full.json')
    builder.build_all_components()
    
    # Create coordinator
    print("\n[Step 2] Creating system coordinator...")
    coordinator = PowerSystemCoordinator(builder)
    
    # Run power flow
    print("\n[Step 3] Solving AC power flow...")
    pf = run_power_flow(builder, coordinator, verbose=True)
    
    print("\n" + "="*70)
    print("   SUCCESS: Kundur system power flow converged!")
    print("="*70)
    print("\nKey Results:")
    results = pf.get_results()
    print(f"  Voltage range: {np.min(results['V']):.4f} - {np.max(results['V']):.4f} pu")
    print(f"  Angle range: {np.degrees(np.min(results['theta'])):.2f} - {np.degrees(np.max(results['theta'])):.2f} deg")
    print(f"  Total generation: {np.sum(results['P'][results['P'] > 0]):.3f} pu")
    print(f"  Total load: {-np.sum(results['P'][results['P'] < 0]):.3f} pu")
    print(f"  Power balance: {np.sum(results['P']):.6f} pu")
    print("="*70)
    
except Exception as e:
    import traceback
    print("\n" + "="*70)
    print("   ERROR: Power flow test failed")
    print("="*70)
    print(f"\nError message: {e}\n")
    traceback.print_exc()
    sys.exit(1)
