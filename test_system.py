"""Test script - Demonstrates modular PyPHS power system"""
import sys
sys.path.insert(0, '/home/claude')
import json
from utils.system_builder import PowerSystemBuilder


def test_component_access(builder):
    """Test accessing individual components"""
    print("\n=== Component Access Test ===")
    
    # Access first generator
    gen0 = builder.generators[0]
    print(f"\nGenerator 0 Core:")
    print(f"  States ({len(gen0.x)}): {[str(s) for s in gen0.x]}")
    print(f"  Hamiltonian: {gen0.H}")
    print(f"  Dissipations: {len(gen0.w)}")
    print(f"  Ports: {len(gen0.u)} inputs, {len(gen0.y)} outputs")
    
    # Access first exciter
    exc0 = builder.exciters[0]
    print(f"\nExciter 0 Core:")
    print(f"  States ({len(exc0.x)}): {[str(s) for s in exc0.x]}")
    print(f"  Hamiltonian: {exc0.H}")
    
    # Access network
    net = builder.network
    print(f"\nNetwork Core:")
    print(f"  States ({len(net.x)}): {[str(s) for s in net.x]}")
    print(f"  Hamiltonian: {net.H}")
    print(f"  Generators connected: {builder.net_metadata['n_gen']}")


def test_hamiltonian_gradients(builder):
    """Test Hamiltonian gradient computation"""
    print("\n=== Hamiltonian Gradient Test ===")
    
    gen0 = builder.generators[0]
    dxH = gen0.dxH()
    
    print(f"\nGenerator 0 gradients:")
    for xi, grad in zip(gen0.x, dxH):
        print(f"  ∂H/∂{xi} = {grad}")


def test_parameter_substitution(builder):
    """Test parameter substitutions"""
    print("\n=== Parameter Substitution Test ===")
    
    gen0 = builder.generators[0]
    print(f"\nGenerator 0 parameters:")
    for key, val in list(gen0.subs.items())[:5]:
        print(f"  {key} = {val}")
    
    exc0 = builder.exciters[0]
    print(f"\nExciter 0 parameters:")
    for key, val in list(exc0.subs.items())[:5]:
        print(f"  {key} = {val}")


def test_metadata_access(builder):
    """Test metadata retrieval"""
    print("\n=== Metadata Access Test ===")
    
    print("\nGenerator metadata:")
    for meta in builder.gen_metadata:
        print(f"  Gen {meta['idx']}: Bus={meta['bus']}, M={meta['M']:.2f}, "
              f"xd={meta['xd']:.4f}, xq={meta['xq']:.4f}")
    
    print("\nExciter metadata:")
    for meta in builder.exc_metadata:
        print(f"  Exc {meta['idx']}: KA={meta['KA']:.1f}, TA={meta['Vref']:.2f}")
    
    print("\nGovernor metadata:")
    for meta in builder.gov_metadata:
        print(f"  Gov {meta['idx']}: R={meta['R']:.4f}, T1={meta['T1']:.2f}")
    
    print("\nNetwork metadata:")
    print(f"  Load powers: {builder.net_metadata['load_P']}")
    print(f"  X_intra: {builder.net_metadata['X_intra']}")
    print(f"  X_tie: {builder.net_metadata['X_tie']}")


def test_system_modification(json_file):
    """Test modifying system configuration"""
    print("\n=== System Modification Test ===")
    
    # Load and modify JSON
    with open(json_file, 'r') as f:
        data = json.load(f)
    
    print(f"\nOriginal: {len(data['GENROU'])} generators")
    
    # Modify generator parameter
    orig_M = data['GENROU'][0]['M']
    data['GENROU'][0]['M'] = orig_M * 1.2
    print(f"Modified Gen 1 M: {orig_M} -> {data['GENROU'][0]['M']}")
    
    # Save temporary file
    temp_file = 'test_cases/Kundur_System/temp_config.json'
    with open(temp_file, 'w') as f:
        json.dump(data, f, indent=2)
    
    # Rebuild system
    new_builder = PowerSystemBuilder(temp_file)
    new_builder.build_all_components()
    
    print(f"Rebuilt system M value: {new_builder.gen_metadata[0]['M']:.2f}")
    
    return new_builder


def main(json_file='test_cases/Kundur_System/kundur_full.json'):
    """Run all tests"""
    print("=" * 60)
    print("MODULAR PyPHS POWER SYSTEM TEST")
    print("=" * 60)
    
    # Build initial system
    builder = PowerSystemBuilder(json_file)
    builder.build_all_components()
    builder.summary()
    
    # Run tests
    test_component_access(builder)
    test_hamiltonian_gradients(builder)
    test_parameter_substitution(builder)
    test_metadata_access(builder)
    test_system_modification(json_file)
    
    print("\n" + "=" * 60)
    print("ALL TESTS COMPLETED")
    print("=" * 60)
    
    return builder


if __name__ == "__main__":
    builder = main()
