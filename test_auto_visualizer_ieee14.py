"""
Test script for Auto-Layout Power System Visualizer - IEEE 14-Bus System

Generates automatic layout visualizations of the IEEE 14-bus system
using impedance-based positioning (no hardcoded coordinates).

The IEEE 14-bus system includes:
- 5 generators (GENROU)
- Mixed exciters (ESST3A + EXST1)
- Mixed governors (IEEEG1 + TGOV1)
- 14 buses with transformers and transmission lines
"""

import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

try:
    from utils.auto_visualizer import AutoPHSVisualizer
except ImportError as e:
    print(f"CRITICAL ERROR: Could not import AutoPHSVisualizer. {e}")
    sys.exit(1)

import json


class MockBuilder:
    """Simulates the PHS Builder environment for system data."""
    def __init__(self, json_path):
        if not os.path.exists(json_path):
            raise FileNotFoundError(f"JSON file not found at: {json_path}")
        with open(json_path, 'r') as f:
            self.system_data = json.load(f)


def run_auto_visualization(spacing_factor=1.5, node_size_factor=1.0,
                           angle_spread=False, strict_component_clearance=False, bus_clearance=0.0,
                           distance_scale='log', min_distance=0.02, max_distance=1.0,
                           load_scale=1.0, generator_scale=1.0, transformer_scale=1.0,
                           exciter_scale=1.0, governor_scale=1.0):
    """
    Generate IEEE 14-bus visualizations with adjustable component spacing and sizes.
    
    Args:
        spacing_factor: Controls component spread around buses (default=1.5)
        node_size_factor: Controls visual size of components (default=1.0)
        load_scale: Scale for load triangles (default=1.0)
        generator_scale: Scale for generator ellipses (default=1.0)
        transformer_scale: Scale for transformer diamonds (default=1.0)
        exciter_scale: Scale for exciter boxes (default=1.0)
        governor_scale: Scale for governor boxes (default=1.0)
    """
    JSON_PATH = 'test_cases/ieee14bus/ieee14_system.json'

    print("\n" + "="*70)
    print("   AUTO-LAYOUT IEEE 14-BUS SYSTEM VISUALIZER")
    print("   Positions computed from electrical distances (no hardcoding)")
    print(f"   Load: {load_scale}x | Generator: {generator_scale}x | Transformer: {transformer_scale}x")
    print(f"   Exciter: {exciter_scale}x | Governor: {governor_scale}x")
    print("="*70)

    try:
        builder = MockBuilder(JSON_PATH)
        viz = AutoPHSVisualizer(builder)

        # NEATO: Best spring model layout for power systems
        print("\n" + "-"*50)
        print("Generating NEATO layout (Professional Quality)...")
        print("-"*50)
        
        generated_files = []
        
        print("\n[NEATO] Spring model layout (RECOMMENDED)")
        output = viz.draw_graphviz(
            filename="outputs/ieee14_graphviz_neato",
            layout_engine="neato",
            format="svg",
            show_impedance=True,
            show_voltage=True,
            load_scale=load_scale,
            generator_scale=generator_scale,
            transformer_scale=transformer_scale,
            exciter_scale=exciter_scale,
            governor_scale=governor_scale
        )
        if output:
            generated_files.append(output)

        # # COMMENTED OUT: Other Graphviz engines (not as accurate)
        # engines = [
        #     ("dot", "Hierarchical layout"),
        #     ("fdp", "Force-directed placement"),
        # ]
        # for engine, description in engines:
        #     print(f"\n[{engine.upper()}] {description}")
        #     output = viz.draw_graphviz(
        #         filename=f"outputs/ieee14_graphviz_{engine}",
        #         layout_engine=engine,
        #         format="svg",
        #         show_impedance=True,
        #         show_voltage=True
        #     )
        #     if output:
        #         generated_files.append(output)

        # # COMMENTED OUT: Original manual SVG generation (not accurate)
        # print("\n" + "-"*50)
        # print("Generating MANUAL SVG layouts (original approach)...")
        # print("-"*50)
        # 
        # print("\n[MANUAL] Hierarchical layout...")
        # viz.draw("outputs/ieee14_manual_hierarchical.svg", 
        #         layout_method="hierarchical",
        #         spacing_factor=spacing_factor,
        #         node_size_factor=node_size_factor,
        #         angle_spread=angle_spread,
        #         strict_component_clearance=strict_component_clearance,
        #         bus_clearance=bus_clearance)
        # generated_files.append("outputs/ieee14_manual_hierarchical.svg")
        #
        # print("\n[MANUAL] Force-directed layout...")
        # viz.draw("outputs/ieee14_manual_force_directed.svg", 
        #         layout_method="force_directed",
        #         spacing_factor=spacing_factor,
        #         node_size_factor=node_size_factor,
        #         angle_spread=angle_spread,
        #         strict_component_clearance=strict_component_clearance,
        #         bus_clearance=bus_clearance)
        # generated_files.append("outputs/ieee14_manual_force_directed.svg")
        #
        # print("\n[MANUAL] Electrical distance layout...")
        # viz.draw_electrical_distance("outputs/ieee14_electrical_distance.svg",
        #                              svg_width=1400, svg_height=900,
        #                              spacing_factor=spacing_factor,
        #                              node_size_factor=node_size_factor,
        #                              angle_spread=angle_spread,
        #                              strict_component_clearance=strict_component_clearance,
        #                              bus_clearance=bus_clearance,
        #                              distance_scale=distance_scale,
        #                              min_distance=min_distance,
        #                              max_distance=max_distance)
        # generated_files.append("outputs/ieee14_electrical_distance.svg")

        print("\n" + "="*70)
        print("[SUCCESS] Generated IEEE 14-bus visualization:")
        for f in generated_files:
            print(f"  ✓ {f}")
        print(f"\n[INFO] Scales - Loads: {load_scale}x | Generators: {generator_scale}x | Transformers: {transformer_scale}x")
        print("[INFO] Using NEATO layout for best accuracy!")
        print("="*70)

    except Exception as e:
        import traceback
        print(f"\n[FAILED] An error occurred: {e}")
        traceback.print_exc()


if __name__ == "__main__":
    # ========== TUNE VISUALIZATION PARAMETERS HERE ==========
    # Individual scale controls for each component type:
    
    LOAD_SCALE = 2.5          # <-- Scale for load triangles
    GENERATOR_SCALE = 1.5     # <-- Scale for generator ellipses
    TRANSFORMER_SCALE = 1.5   # <-- Scale for transformer diamonds
    EXCITER_SCALE = 1.5       # <-- Scale for exciter boxes
    GOVERNOR_SCALE = 1.5      # <-- Scale for governor boxes
    
    # Other parameters (usually don't need to change these)
    SPACING_FACTOR = 1.5
    NODE_SIZE_FACTOR = 1.0
    ANGLE_SPREAD = False
    STRICT_CLEARANCE = False
    BUS_CLEARANCE = 0.0
    DISTANCE_SCALE = 'log'
    MIN_DISTANCE = 0.02
    MAX_DISTANCE = 1.0
    
    run_auto_visualization(
        spacing_factor=SPACING_FACTOR,
        node_size_factor=NODE_SIZE_FACTOR,
        angle_spread=ANGLE_SPREAD,
        strict_component_clearance=STRICT_CLEARANCE,
        bus_clearance=BUS_CLEARANCE,
        distance_scale=DISTANCE_SCALE,
        min_distance=MIN_DISTANCE,
        max_distance=MAX_DISTANCE,
        load_scale=LOAD_SCALE,
        generator_scale=GENERATOR_SCALE,
        transformer_scale=TRANSFORMER_SCALE,
        exciter_scale=EXCITER_SCALE,
        governor_scale=GOVERNOR_SCALE
    )
