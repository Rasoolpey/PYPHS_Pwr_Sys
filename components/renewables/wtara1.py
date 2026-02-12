"""
WTARA1 Wind Turbine Aerodynamics Model - Port-Hamiltonian Formulation

Simplified aerodynamics model without detailed wind speed modeling.
Purely algebraic - no dynamic states.

Pm = Pe0 - (theta - theta0) * theta * Ka + Pe0
(The Andes formulation sums to: Pm = 2*Pe0 - Ka*theta*(theta - theta0))

At equilibrium (theta = theta0): Pm = Pe0 (matches electrical power)
When pitch increases: Pm decreases (power shedding)

Reference: WECC WTARA1 model specification
"""
from utils.pyphs_core import DynamicsCore
import numpy as np


def wtara1_compute_pm(theta, meta):
    """
    Compute mechanical power from pitch angle.

    Args:
        theta: pitch angle in radians
        meta: dict with parameters

    Returns:
        Pm: mechanical power (pu)
    """
    Pe0 = meta['Pe0']
    theta0 = meta['theta0r']  # initial pitch in radians
    Ka = meta['Ka']

    # Aerodynamic power: reduces with pitch angle deviation
    Pm = Pe0 - Ka * theta * (theta - theta0)
    return Pm


def build_wtara1_core(aero_data, S_system=100.0):
    """Build WTARA1 aerodynamics as DynamicsCore (algebraic only).

    Args:
        aero_data: dict with aerodynamics parameters from JSON
        S_system: system base power (MVA)

    Returns:
        core: DynamicsCore object (0 states)
        metadata: dict with additional info
    """
    core = DynamicsCore(label=f'WTARA1_{aero_data["idx"]}')

    Ka = aero_data.get('Ka', 1.0)
    theta0_deg = aero_data.get('theta0', 0.0)
    theta0r = np.radians(theta0_deg)

    metadata = {
        'idx': aero_data['idx'],
        'rego': aero_data['rego'],
        'Ka': Ka,
        'theta0': theta0_deg,
        'theta0r': theta0r,
        'Pe0': 0.0,  # Set during initialization from power flow
    }

    core.set_metadata(metadata)
    core.n_states = 0
    core.output_fn = lambda x, ports, meta: {
        'Paero': wtara1_compute_pm(ports.get('theta', meta['theta0r']), meta)
    }
    core.component_type = "renewable"
    core.model_name = "WTARA1"

    return core, metadata
