import os
import sys
import json

# Ensure the local 'utils' folder is in the path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

try:
    from utils.visualizer import PHSVisualizer
except ImportError as e:
    print(f"CRITICAL ERROR: Could not import PHSVisualizer. {e}")
    sys.exit(1)

class MockBuilder:
    """Simulates the PHS Builder environment for your thesis data."""
    def __init__(self, json_path):
        if not os.path.exists(json_path):
            raise FileNotFoundError(f"JSON file not found at: {json_path}")
        with open(json_path, 'r') as f:
            self.system_data = json.load(f)

def run_full_extraction_and_plot():
    # Path to your specific Kundur JSON
    JSON_PATH = 'test_cases/Kundur_System/kundur_full.json'
    
    print("\n" + "="*60)
    print("   PHS SYSTEM EXTRACTION & VISUALIZATION MODULE")
    print("="*60)

    try:
        # 1. Initialize the environment
        builder = MockBuilder(JSON_PATH)
        viz = PHSVisualizer(builder)

        # 2. Extract and Plot
        # This calls the draw method which handles the full detail dump internally
        viz.draw("outputs/kundur_system_figure.svg")

        print("\n[COMPLETE] Check 'outputs/kundur_system_figure.svg'")
        print("="*60)

    except Exception as e:
        print(f"\n[FAILED] An error occurred during extraction: {e}")

if __name__ == "__main__":
    run_full_extraction_and_plot()