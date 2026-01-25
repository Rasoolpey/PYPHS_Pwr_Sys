"""
Modular PyPHS Power System - Demo
Shows how to work with the modular structure
"""
import sys
sys.path.insert(0, '/home/claude')
import json
from utils.system_builder import PowerSystemBuilder
from utils.component_factory import ComponentFactory


def demo_basic_usage():
    """Basic usage - build system from JSON"""
    print("=== BASIC USAGE ===\n")
    
    builder = PowerSystemBuilder('test_cases/Kundur_System/kundur_full.json')
    builder.build_all_components()
    builder.summary()


def demo_access_components():
    """Access individual components"""
    print("\n=== ACCESSING COMPONENTS ===\n")
    
    builder = PowerSystemBuilder('test_cases/Kundur_System/kundur_full.json')
    builder.build_all_components()
    
    # Access generators
    print("Generators:")
    for i, gen in enumerate(builder.generators):
        print(f"  Gen {i}: {len(gen.x)} states, Hamiltonian: {gen.H}")
    
    # Access controls
    print("\nControls:")
    print(f"  Exciters: {len(builder.exciters)}")
    print(f"  Governors: {len(builder.governors)}")
    
    # Access network
    print(f"\nNetwork: {len(builder.network.x)} states")


def demo_modify_parameters():
    """Modify parameters dynamically"""
    print("\n=== MODIFYING PARAMETERS ===\n")
    
    # Load JSON
    with open('test_cases/Kundur_System/kundur_full.json', 'r') as f:
        data = json.load(f)
    
    # Modify generator inertia
    print("Original Gen 1 M:", data['GENROU'][0]['M'])
    data['GENROU'][0]['M'] = 15.0
    print("Modified Gen 1 M:", data['GENROU'][0]['M'])
    
    # Save and rebuild
    temp_file = 'test_cases/Kundur_System/modified_system.json'
    with open(temp_file, 'w') as f:
        json.dump(data, f, indent=2)
    
    builder = PowerSystemBuilder(temp_file)
    builder.build_all_components()
    print("Rebuilt system M:", builder.gen_metadata[0]['M'])


def demo_add_generator():
    """Show how to add a new generator (conceptual)"""
    print("\n=== ADDING NEW GENERATOR TYPE ===\n")
    
    print("To add a new generator type (e.g., GENSAL):")
    print("1. Create components/generators/gensal.py")
    print("2. Implement build_gensal_core(gen_data, S_system)")
    print("3. Register in ComponentFactory:")
    print("   factory.register_model('generators', 'GENSAL', 'gensal')")
    print("4. Add GENSAL data to JSON file")
    print("5. System will automatically use new model")


def main():
    """Run all demos"""
    demo_basic_usage()
    demo_access_components()
    demo_modify_parameters()
    demo_add_generator()
    
    print("\n" + "="*60)
    print("Demo complete!")
    print("="*60)


if __name__ == "__main__":
    main()
