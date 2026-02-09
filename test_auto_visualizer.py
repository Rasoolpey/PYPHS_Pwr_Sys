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


def run_auto_visualization():
    JSON_PATH = 'test_cases/Kundur_System/kundur_full.json'

    print("\n" + "="*70)
    print("   AUTO-LAYOUT POWER SYSTEM VISUALIZER")
    print("   Using Professional Graph Layout (VS Code Debug Visualizer Style)")
    print("="*70)

    try:
        builder = MockBuilder(JSON_PATH)
        viz = AutoPHSVisualizer(builder)

        # NEW: Generate professional Graphviz visualizations (like VS Code Debug Visualizer)
        print("\n" + "-"*50)
        print("Generating GRAPHVIZ layouts (Professional Quality)...")
        print("-"*50)
        
        # Try different Graphviz engines
        engines = [
            ("dot", "Hierarchical layout (best for power systems)"),
            ("neato", "Spring model layout"),
            ("fdp", "Force-directed placement"),
        ]
        
        generated_files = []
        for engine, description in engines:
            print(f"\n[{engine.upper()}] {description}")
            output = viz.draw_graphviz(
                filename=f"outputs/graphviz_{engine}",
                layout_engine=engine,
                format="svg",
                show_impedance=True,
                show_voltage=True
            )
            if output:
                generated_files.append(output)

        # OLD: Original manual SVG generation (for comparison)
        print("\n" + "-"*50)
        print("Generating MANUAL SVG layouts (original approach)...")
        print("-"*50)
        
        print("\n[MANUAL] Hierarchical layout...")
        viz.draw("outputs/manual_hierarchical.svg", layout_method="hierarchical", 
                spacing_factor=2.0, strict_component_clearance=True)
        generated_files.append("outputs/manual_hierarchical.svg")
        
        print("\n[MANUAL] Force-directed layout...")
        viz.draw("outputs/manual_force_directed.svg", layout_method="force_directed",
                spacing_factor=2.0, strict_component_clearance=True)
        generated_files.append("outputs/manual_force_directed.svg")

        print("\n" + "="*70)
        print("[SUCCESS] Generated visualizations:")
        for f in generated_files:
            print(f"  ✓ {f}")
        print("\n[RECOMMENDED] Open the Graphviz outputs for VS Code-style quality!")
        print("="*70)

    except Exception as e:
        import traceback
        print(f"\n[FAILED] An error occurred: {e}")
        traceback.print_exc()


if __name__ == "__main__":
    run_auto_visualization()
