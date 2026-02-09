"""
Test script for Auto-Layout Power System Visualizer

This script demonstrates the automatic layout visualization that computes
component positions based on electrical distances (impedances) rather than
using hardcoded coordinates.

Two layout methods are available:
1. hierarchical - Separates by voltage level and area, good for radial systems
2. force_directed - Spring-electrical model based on impedances
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


def run_auto_visualization(load_scale=1.0, generator_scale=1.0, transformer_scale=1.0,
                           exciter_scale=1.0, governor_scale=1.0):
    """
    Generate Kundur system visualizations.
    
    Args:
        load_scale: Scale for load triangles (default=1.0)
        generator_scale: Scale for generator ellipses (default=1.0)
        transformer_scale: Scale for transformer diamonds (default=1.0)
        exciter_scale: Scale for exciter boxes (default=1.0)
        governor_scale: Scale for governor boxes (default=1.0)
    """
    JSON_PATH = 'test_cases/Kundur_System/kundur_full.json'

    print("\n" + "="*70)
    print("   AUTO-LAYOUT POWER SYSTEM VISUALIZER")
    print("   Using Professional Graph Layout (VS Code Debug Visualizer Style)")
    print(f"   Load Scale: {load_scale}x | Generator Scale: {generator_scale}x")
    print(f"   Transformer Scale: {transformer_scale}x | Exciter/Governor: {exciter_scale}x/{governor_scale}x")
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
            filename="outputs/kundur_graphviz_neato",
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
        #         filename=f"outputs/graphviz_{engine}",
        #         layout_engine=engine,
        #         format="svg",
        #         show_impedance=True,
        #         show_voltage=True
        #     )
        #     if output:
        #         generated_files.append(output)

        # # COMMENTED OUT: Manual SVG generation (hierarchical/force-directed layouts)
        # print("\n" + "-"*50)
        # print("Generating MANUAL SVG layouts with colored busbars...")
        # print("-"*50)
        # 
        # print("\n[MANUAL] Hierarchical layout...")
        # viz.draw("outputs/kundur_manual_hierarchical.svg", layout_method="hierarchical", 
        #         spacing_factor=2.0, strict_component_clearance=True)
        # generated_files.append("outputs/kundur_manual_hierarchical.svg")
        # 
        # print("\n[MANUAL] Force-directed layout...")
        # viz.draw("outputs/kundur_manual_force_directed.svg", layout_method="force_directed",
        #         spacing_factor=2.0, strict_component_clearance=True)
        # generated_files.append("outputs/kundur_manual_force_directed.svg")

        print("\n" + "="*70)
        print("[SUCCESS] Generated Kundur System visualization:")
        for f in generated_files:
            print(f"  ✓ {f}")
        print(f"\n[INFO] Scale factors - Loads: {load_scale}x | Generators: {generator_scale}x | Transformers: {transformer_scale}x")
        print("[INFO] Graphviz NEATO layout - Voltage-color-coded busbars:")
        print("       - Dark Red/Orange: EHV/HV (≥138kV) | Yellow/Gold: MHV (69-137kV)")
        print("       - Green: MV (30-68kV) | Blue: LV (<30kV)")
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
    
    # Examples:
    # LOAD_SCALE = 0.5       # Make loads half size
    # TRANSFORMER_SCALE = 1.5  # Make transformers 50% larger
    # GENERATOR_SCALE = 0.8    # Make generators 20% smaller
    
    run_auto_visualization(
        load_scale=LOAD_SCALE,
        generator_scale=GENERATOR_SCALE,
        transformer_scale=TRANSFORMER_SCALE,
        exciter_scale=EXCITER_SCALE,
        governor_scale=GOVERNOR_SCALE
    )
