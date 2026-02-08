"""BusFreq - Bus Frequency Measurement Device

Port-Hamiltonian implementation of frequency measurement at a bus.

The frequency is computed from the rate of change of voltage phase angle,
then filtered to provide a smooth frequency signal for PSS and other controls.

Signal path:
    Bus voltage angle θ -> dθ/dt calculation
    -> f = f_nominal * (1 + dθ/dt)
    -> [1/(1+sTf) filter]
    -> [sTw/(1+sTw) washout (optional)]
    -> frequency output
"""

import numpy as np
from utils.pyphs_core import DynamicsCore


def busfreq_dynamics(x, ports, meta):
    """
    BusFreq frequency measurement dynamics.

    States: [xf, xw]
        xf: Frequency filter state (low-pass filter)
        xw: Washout filter state (high-pass filter, optional)

    Args:
        x: numpy array [xf, xw]
        ports: dict with 'theta' (bus voltage angle in radians)
        meta: dict of parameters

    Returns:
        x_dot: numpy array of state derivatives
    """
    # Extract states
    xf, xw = x

    # Extract ports
    theta = ports.get('theta', 0.0)  # Bus voltage angle
    theta_prev = ports.get('theta_prev', 0.0)  # Previous angle for derivative
    dt = ports.get('dt', 0.001)  # Time step for derivative calculation
    
    # Alternative: direct dtheta/dt input
    dtheta_dt = ports.get('dtheta_dt', None)

    # Extract parameters
    Tf = meta['Tf']
    Tw = meta['Tw']
    fn = meta['fn']  # Nominal frequency (Hz)
    omega_n = 2.0 * np.pi * fn  # Nominal angular frequency (rad/s)

    # Calculate frequency deviation from angle derivative
    # f = fn * (1 + dθ/dt / ω_n)
    # In per-unit: f_pu = 1 + dθ/dt / ω_n
    
    if dtheta_dt is not None:
        # Direct derivative provided
        f_measured = 1.0 + dtheta_dt / omega_n
    else:
        # Finite difference approximation
        if dt > 1e-6:
            dtheta = theta - theta_prev
            # Handle angle wrapping
            if dtheta > np.pi:
                dtheta -= 2.0 * np.pi
            elif dtheta < -np.pi:
                dtheta += 2.0 * np.pi
            dtheta_dt_calc = dtheta / dt
            f_measured = 1.0 + dtheta_dt_calc / omega_n
        else:
            f_measured = 1.0  # No change

    # Initialize derivatives
    x_dot = np.zeros(2)

    # 1. Low-pass filter: d(xf)/dt = (f_measured - xf) / Tf
    if Tf > 1e-6:
        x_dot[0] = (f_measured - xf) / Tf
    else:
        x_dot[0] = 0.0
        xf = f_measured

    # 2. Washout (optional): d(xw)/dt = (xf - xw) / Tw
    # Output: f_out = Tw * (xf - xw) / Tw = (xf - xw)
    # This provides high-pass filtered frequency deviation
    if Tw > 1e-6:
        x_dot[1] = (xf - xw) / Tw
    else:
        x_dot[1] = 0.0

    return x_dot


def busfreq_output(x, ports, meta):
    """
    Compute frequency measurement output.

    Args:
        x: numpy array [xf, xw]
        ports: dict with bus voltage angle
        meta: dict of parameters

    Returns:
        f: Measured frequency (p.u.)
    """
    xf, xw = x

    # Extract parameters
    Tw = meta['Tw']
    use_washout = meta.get('use_washout', False)

    # Output is filtered frequency
    if use_washout and Tw > 1e-6:
        # With washout: output frequency deviation
        f = xf - xw
    else:
        # Without washout: output absolute frequency
        f = xf

    return f


def build_busfreq_core(busfreq_data, initial_conditions=None):
    """
    Build BusFreq measurement device as DynamicsCore.

    Args:
        busfreq_data: dict with BusFreq parameters from JSON
        initial_conditions: dict with initial state values (optional)

    Returns:
        DynamicsCore object configured for BusFreq
    """
    # Extract and validate parameters
    Tf = max(busfreq_data.get('Tf', 0.02), 0.001)
    Tw = max(busfreq_data.get('Tw', 0.02), 0.001)
    fn = busfreq_data.get('fn', 60.0)  # Nominal frequency
    bus_idx = busfreq_data.get('bus', 1)

    # Build metadata dictionary
    metadata = {
        'idx': busfreq_data.get('idx', 'BusFreq_1'),
        'name': busfreq_data.get('name', 'BusFreq_1'),
        'bus': bus_idx,
        'Tf': Tf,
        'Tw': Tw,
        'fn': fn,
        'use_washout': False  # Typically False for standard frequency measurement
    }

    # Initialize states: [xf, xw]
    if initial_conditions is not None:
        x0 = np.array([
            initial_conditions.get('xf', 1.0),
            initial_conditions.get('xw', 1.0)
        ])
    else:
        # Default initialization at nominal frequency (1.0 p.u.)
        xf0 = 1.0
        xw0 = 1.0
        x0 = np.array([xf0, xw0])

    # Create DynamicsCore
    core = DynamicsCore(
        label=f"BusFreq_{metadata['idx']}",
        dynamics_fn=busfreq_dynamics
    )

    # Set metadata and component attributes
    core.set_metadata(metadata)
    core.n_states = 2
    core.output_fn = busfreq_output
    core.component_type = "measurement"
    core.model_name = "BusFreq"

    return core, metadata


def compute_initial_states(theta0, omega0, params):
    """
    Compute equilibrium initial states for BusFreq.

    At equilibrium with constant frequency:
        xf = omega0 (steady-state frequency)
        xw = omega0 (washout equilibrium)

    Args:
        theta0: Initial bus voltage angle (rad)
        omega0: Initial frequency (p.u., typically 1.0)
        params: BusFreq parameter dict

    Returns:
        x0: numpy array [xf, xw]
    """
    # At equilibrium, both states equal the steady-state frequency
    xf = omega0
    xw = omega0

    x0 = np.array([xf, xw])

    return x0


# Example usage and testing
if __name__ == "__main__":
    # Test BusFreq initialization
    test_params = {
        'idx': 'BusFreq_test',
        'name': 'BusFreq_test',
        'bus': 1.0,
        'Tf': 0.02,
        'Tw': 0.02,
        'fn': 60.0
    }

    # Build frequency measurement
    busfreq = build_busfreq_core(test_params)

    # Test dynamics at nominal conditions
    test_ports = {
        'theta': 0.0,
        'theta_prev': 0.0,
        'dt': 0.001,
        'dtheta_dt': 0.0  # No frequency deviation
    }
    x_dot = busfreq.dynamics_fn(busfreq.states, test_ports, busfreq.meta)
    f = busfreq.output_fn(busfreq.states, test_ports, busfreq.meta)

    print("BusFreq Measurement Test")
    print(f"States: {busfreq.states}")
    print(f"Derivatives: {x_dot}")
    print(f"Output f: {f}")
    print(f"Max |dx/dt|: {np.max(np.abs(x_dot))}")

    # Test with frequency deviation
    test_ports2 = {
        'theta': 0.01,
        'theta_prev': 0.0,
        'dt': 0.001,
        'dtheta_dt': 0.01 / 0.001  # Small frequency increase
    }
    x_dot2 = busfreq.dynamics_fn(busfreq.states, test_ports2, busfreq.meta)
    f2 = busfreq.output_fn(busfreq.states, test_ports2, busfreq.meta)

    print("\nWith Frequency Deviation:")
    print(f"Derivatives: {x_dot2}")
    print(f"Output f: {f2}")
