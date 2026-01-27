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
    print("   Positions computed from electrical distances (no hardcoding)")
    print("="*70)

    try:
        builder = MockBuilder(JSON_PATH)
        viz = AutoPHSVisualizer(builder)

        # Generate visualizations with different layout methods
        print("\n" + "-"*50)
        print("Generating HIERARCHICAL layout...")
        print("-"*50)
        viz.draw("outputs/auto_layout_hierarchical.svg", layout_method="hierarchical")

        print("\n" + "-"*50)
        print("Generating FORCE-DIRECTED layout...")
        print("-"*50)
        viz.draw("outputs/auto_layout_force_directed.svg", layout_method="force_directed")

        print("\n" + "="*70)
        print("[SUCCESS] Generated 2 auto-layout visualizations:")
        print("  - outputs/auto_layout_hierarchical.svg")
        print("  - outputs/auto_layout_force_directed.svg")
        print("="*70)

    except Exception as e:
        import traceback
        print(f"\n[FAILED] An error occurred: {e}")
        traceback.print_exc()


if __name__ == "__main__":
    run_auto_visualization()
