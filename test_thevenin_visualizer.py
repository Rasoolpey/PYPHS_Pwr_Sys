"""Test Auto-Layout Visualizer for Thevenin Model"""
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


def run_thevenin_visualization():
    JSON_PATH = 'test_cases/Thevenin_model/ieee_based_system.json'

    print("\n" + "="*70)
    print("   AUTO-LAYOUT VISUALIZATION FOR THEVENIN MODEL")
    print("   Positions computed from electrical distances")
    print("="*70)

    try:
        builder = MockBuilder(JSON_PATH)
        viz = AutoPHSVisualizer(builder)

        # Generate visualization with hierarchical layout
        print("\n" + "-"*50)
        print("Generating HIERARCHICAL layout...")
        print("-"*50)
        viz.draw("outputs/thevenin_layout_hierarchical.svg", layout_method="hierarchical")

        print("\n" + "-"*50)
        print("Generating FORCE-DIRECTED layout...")
        print("-"*50)
        viz.draw("outputs/thevenin_layout_force_directed.svg", layout_method="force_directed")

        print("\n" + "="*70)
        print("[SUCCESS] Generated 2 auto-layout visualizations:")
        print("  - outputs/thevenin_layout_hierarchical.svg")
        print("  - outputs/thevenin_layout_force_directed.svg")
        print("="*70)

    except Exception as e:
        import traceback
        print(f"\n[FAILED] An error occurred: {e}")
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    run_thevenin_visualization()
