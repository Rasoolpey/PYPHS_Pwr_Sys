"""Main System - Connects all PyPHS components"""
import sys
sys.path.insert(0, '/home/claude')

from utils.system_builder import PowerSystemBuilder


def main(json_file):
    """Build and test power system from JSON
    
    Args:
        json_file: path to configuration JSON
    """
    # Build system
    builder = PowerSystemBuilder(json_file)
    builder.build_all_components()
    builder.summary()
    
    # Show component details
    print("\nGenerator Details:")
    for i, meta in enumerate(builder.gen_metadata):
        print(f"  Gen {meta['idx']}: Bus {meta['bus']}, Sn={meta['Sn']} MVA, M={meta['M']:.2f}")
    
    print("\nControl Mapping:")
    control_map = builder.get_control_mapping()
    for gen_idx, exc_idx in control_map['exciters'].items():
        print(f"  Gen {gen_idx} -> Exciter {exc_idx}")
    for gen_idx, gov_idx in control_map['governors'].items():
        print(f"  Gen {gen_idx} -> Governor {gov_idx}")
    
    print("\nNetwork Configuration:")
    net_meta = builder.net_metadata
    print(f"  Generators: {net_meta['n_gen']}")
    print(f"  Lines: {len(net_meta['line_configs'])}")
    for line_cfg in net_meta['line_configs']:
        print(f"    {line_cfg['name']}: Connects Gen {line_cfg['gen_idx']}")
    
    return builder


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1:
        json_file = sys.argv[1]
    else:
        json_file = 'test_cases/Kundur_System/kundur_full.json'
    
    system = main(json_file)
