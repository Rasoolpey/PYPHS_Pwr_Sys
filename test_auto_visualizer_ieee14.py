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
                           distance_scale='log', min_distance=0.02, max_distance=1.0):
    """
    Generate IEEE 14-bus visualizations with adjustable component spacing and sizes.
    
    Args:
        spacing_factor: Controls component spread around buses (default=1.5)
                       - 1.0 = default spacing
                       - 1.5 = recommended for IEEE 14-bus (prevents overlaps)
                       - 2.0 = maximum spread for very crowded diagrams
        node_size_factor: Controls visual size of components (default=1.0)
                         - 1.0 = default size
                         - 0.7 = recommended for compact diagrams
                         - 0.5 = minimum readable size
    """
    JSON_PATH = 'test_cases/ieee14bus/ieee14_system.json'

    print("\n" + "="*70)
    print("   AUTO-LAYOUT IEEE 14-BUS SYSTEM VISUALIZER")
    print("   Positions computed from electrical distances (no hardcoding)")
    print(f"   Component Spacing Factor: {spacing_factor}x")
    print(f"   Node Size Factor: {node_size_factor}x")
    print(f"   Angle Spread: {angle_spread}")
    print(f"   Strict Clearance: {strict_component_clearance}")
    print(f"   Bus Clearance: {bus_clearance}")
    print("="*70)

    try:
        builder = MockBuilder(JSON_PATH)
        viz = AutoPHSVisualizer(builder)

        # Generate visualizations with different layout methods
        print("\n" + "-"*50)
        print(f"Generating HIERARCHICAL layout (spacing={spacing_factor}x, size={node_size_factor}x)...")
        print("-"*50)
        viz.draw("outputs/ieee14_auto_layout_hierarchical.svg", 
                layout_method="hierarchical",
                spacing_factor=spacing_factor,
            node_size_factor=node_size_factor,
            angle_spread=angle_spread,
            strict_component_clearance=strict_component_clearance,
            bus_clearance=bus_clearance)

        print("\n" + "-"*50)
        print(f"Generating FORCE-DIRECTED layout (spacing={spacing_factor}x, size={node_size_factor}x)...")
        print("-"*50)
        viz.draw("outputs/ieee14_auto_layout_force_directed.svg", 
                layout_method="force_directed",
                spacing_factor=spacing_factor,
            node_size_factor=node_size_factor,
            angle_spread=angle_spread,
            strict_component_clearance=strict_component_clearance,
            bus_clearance=bus_clearance)

        print("\n" + "-"*50)
        print(f"Generating ELECTRICAL DISTANCE layout (spacing={spacing_factor}x, size={node_size_factor}x)...")
        print(f"  Distance scaling: {distance_scale} | Min: {min_distance} | Max: {max_distance}")
        print("-"*50)
        viz.draw_electrical_distance("outputs/ieee14_electrical_distance.svg",
                                     svg_width=1400, svg_height=900,
                                     spacing_factor=spacing_factor,
                                     node_size_factor=node_size_factor,
                                     angle_spread=angle_spread,
                                     strict_component_clearance=strict_component_clearance,
                                     bus_clearance=bus_clearance,
                                     distance_scale=distance_scale,
                                     min_distance=min_distance,
                                     max_distance=max_distance)

        print("\n" + "="*70)
        print("[SUCCESS] Generated 3 IEEE 14-bus auto-layout visualizations:")
        print("  - outputs/ieee14_auto_layout_hierarchical.svg")
        print("  - outputs/ieee14_auto_layout_force_directed.svg")
        print("  - outputs/ieee14_electrical_distance.svg")
        print(f"\n  Component spacing factor: {spacing_factor}x")
        print(f"  Node size factor: {node_size_factor}x")
        print("  (Adjust SPACING_FACTOR and NODE_SIZE_FACTOR to fine-tune)")
        print("="*70)

    except Exception as e:
        import traceback
        print(f"\n[FAILED] An error occurred: {e}")
        traceback.print_exc()


if __name__ == "__main__":
    # ========== TUNE VISUALIZATION PARAMETERS HERE ==========
    # SPACING_FACTOR: Controls how far apart components are positioned
    #   - 1.0 = default spacing (may have some overlaps)
    #   - 1.5 = recommended (good balance, prevents most overlaps)
    #   - 2.0 = maximum spread (for very crowded diagrams)
    
    # NODE_SIZE_FACTOR: Controls the visual size of components
    #   - 1.0 = default size
    #   - 0.7 = recommended for compact diagrams
    #   - 0.5 = minimum readable size
    #   - USE SMALLER VALUES to reduce overlaps without spreading the entire graph
    
    SPACING_FACTOR = 1.5  # <-- Adjust component positioning spread
    NODE_SIZE_FACTOR = 1.0  # <-- Adjust component visual sizes
    ANGLE_SPREAD = False  # <-- Spread incident edges around high-degree buses
    STRICT_CLEARANCE = False  # <-- Enforce no overlaps between all components
    BUS_CLEARANCE = 0.0  # <-- Extra spacing between buses
    
    # Distance scaling parameters (for electrical distance layout only)
    DISTANCE_SCALE = 'log'  # <-- 'linear', 'log', or 'sqrt'
    MIN_DISTANCE = 0.02  # <-- Minimum distance (prevents buses too close) - For log: use 0.02-0.05
    MAX_DISTANCE = 1.0  # <-- Maximum distance (prevents buses too far) - For log: use 0.8-1.5
    
    run_auto_visualization(
        spacing_factor=SPACING_FACTOR,
        node_size_factor=NODE_SIZE_FACTOR,
        angle_spread=ANGLE_SPREAD,
        strict_component_clearance=STRICT_CLEARANCE,
        bus_clearance=BUS_CLEARANCE,
        distance_scale=DISTANCE_SCALE,
        min_distance=MIN_DISTANCE,
        max_distance=MAX_DISTANCE
    )
