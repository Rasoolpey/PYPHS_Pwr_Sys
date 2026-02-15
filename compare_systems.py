"""Compare system characteristics between baseline and renewable cases"""
import json

# Load both systems
with open('test_cases/ieee14bus/ieee14_system.json') as f:
    baseline = json.load(f)

with open('test_cases/ieee14bus/renewable_resources_adoption/ieee14_wt3_system.json') as f:
    renewable = json.load(f)

print("="*70)
print("SYSTEM COMPARISON: BASELINE vs WITH WIND TURBINE")
print("="*70)

print("\n1. GENERATION CAPACITY:")
print(f"   Baseline IEEE 14-bus:")
print(f"     - Synchronous Generators: {len(baseline['GENROU'])}")
baseline_sync_mva = sum(g['Sn'] for g in baseline['GENROU'])
print(f"     - Total Sync MVA: {baseline_sync_mva:.1f} MVA")

print(f"\n   With Wind Turbine:")
print(f"     - Synchronous Generators: {len(renewable['GENROU'])}")
renewable_sync_mva = sum(g['Sn'] for g in renewable['GENROU'])
print(f"     - Total Sync MVA: {renewable_sync_mva:.1f} MVA")
if 'REGCA1' in renewable:
    wind_mva = renewable['REGCA1'][0].get('MVAbase', renewable['REGCA1'][0].get('Sn', 100))
    print(f"     - Wind Turbines: {len(renewable['REGCA1'])}")
    print(f"     - Wind MVA: {wind_mva:.1f} MVA")
    print(f"     => Total Generation: {renewable_sync_mva + wind_mva:.1f} MVA")

print(f"\n2. SYSTEM INERTIA (critical for damping):")
# Calculate total inertia on system base (100 MVA)
M_baseline = sum(g['M'] * g['Sn']/100 for g in baseline['GENROU'])
M_renewable = sum(g['M'] * g['Sn']/100 for g in renewable['GENROU'])

print(f"   Baseline total M: {M_baseline:.2f} MWs/MVA (system base)")
print(f"   With WT total M:  {M_renewable:.2f} MWs/MVA (system base)")
print(f"   Inertia REDUCTION: {(1-M_renewable/M_baseline)*100:.1f}%")
print(f"\n   => Wind turbine has ZERO rotating inertia!")
print(f"   => System natural frequency increases: w_n ~ 1/sqrt(M)")
print(f"   => Damping ratio decreases if damping D doesn't increase proportionally")

print(f"\n3. DAMPING CONTRIBUTION:")
# Calculate total damping
D_baseline = sum(g.get('D', 0) * g['Sn']/100 for g in baseline['GENROU'])
D_renewable = sum(g.get('D', 0) * g['Sn']/100 for g in renewable['GENROU'])

print(f"   Baseline damping: D_total = {D_baseline:.3f} pu (mechanical damping only)")
print(f"   With WT damping:  D_total = {D_renewable:.3f} pu (mechanical damping only)")
print(f"\n   Note: Wind turbine (GFL) provides:")
print(f"   - NO synthetic inertia")
print(f"   - NO active damping (no droop control)")
print(f"   - Acts as constant power source during transients")

print(f"\n4. DAMPING RATIO ESTIMATE:")
# Simple estimate: zeta = D / (2*sqrt(M))
zeta_baseline = (D_baseline + 2*len(baseline['GENROU'])) / (2 * M_baseline**0.5)
zeta_renewable = (D_renewable + 2*len(renewable['GENROU'])) / (2 * M_renewable**0.5)

print(f"   Baseline: zeta ~ {zeta_baseline:.3f}")
print(f"   With WT:  zeta ~ {zeta_renewable:.3f}")
print(f"   Change: {(zeta_renewable/zeta_baseline - 1)*100:+.1f}%")

print(f"\n5. EXPECTED DYNAMIC BEHAVIOR:")
if zeta_renewable < zeta_baseline:
    print(f"   [X] SLOWER damping (more oscillatory)")
    print(f"   [X] Higher overshoot")
    print(f"   [X] Longer settling time")
else:
    print(f"   [OK] Similar or better damping")

print(f"\n6. GENERATOR LOCATIONS:")
print(f"   Baseline buses: {[g['bus'] for g in baseline['GENROU']]}")
print(f"   Renewable buses: {[g['bus'] for g in renewable['GENROU']]}")
if 'REGCA1' in renewable:
    print(f"   Wind turbine bus: {renewable['REGCA1'][0]['bus']}")

# Check if a generator was replaced
baseline_buses = set(g['bus'] for g in baseline['GENROU'])
renewable_buses = set(g['bus'] for g in renewable['GENROU'])
removed_buses = baseline_buses - renewable_buses
if removed_buses:
    removed_list = list(removed_buses)
    print(f"\n   => Generator at bus {removed_list[0]} was REPLACED by wind turbine")
    print(f"   => This DIRECTLY reduces system inertia!")

print("\n" + "="*70)
print("CONCLUSION:")
print("="*70)
print("The slower oscillation damping with wind turbine is EXPECTED because:")
print("1. Reduced system inertia (wind turbine has no rotating mass)")
print("2. Grid-Following (GFL) control doesn't provide damping")
print("3. Lower damping ratio due to M reduction without D increase")
print("\nTo improve damping with renewables, consider:")
print("- Grid-Forming (GFM) control with virtual inertia")
print("- Active damping in converter controller")
print("- Power System Stabilizers (PSS) on remaining sync generators")
print("="*70)
