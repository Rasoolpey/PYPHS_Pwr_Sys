"""Standalone PyPHS-compatible Core class"""
import sympy as sp
import numpy as np


class Core:
    """Minimal PyPHS Core implementation for power systems"""
    
    def __init__(self, label='system'):
        self.label = label
        self.x = []  # State variables
        self.w = []  # Dissipative variables
        self.z = []  # Dissipation functions
        self.u = []  # Input ports
        self.y = []  # Output ports
        self.H = 0   # Hamiltonian
        self.subs = {}  # Parameter substitutions
    
    def symbols(self, names):
        """Create sympy symbols (real-valued)"""
        if isinstance(names, str):
            return sp.Symbol(names, real=True)
        elif isinstance(names, list):
            return [sp.Symbol(name, real=True) for name in names]
        else:
            return sp.symbols(names, real=True)
    
    def add_storages(self, states, hamiltonian):
        """Add storage components
        
        Args:
            states: list of state variables or single state
            hamiltonian: energy function
        """
        if not isinstance(states, list):
            states = [states]
        self.x.extend(states)
        self.H += hamiltonian
    
    def add_dissipations(self, w_vars, z_funcs):
        """Add dissipative components
        
        Args:
            w_vars: list of dissipative variables or single variable
            z_funcs: list of dissipation functions or single function
        """
        if not isinstance(w_vars, list):
            w_vars = [w_vars]
        if not isinstance(z_funcs, list):
            z_funcs = [z_funcs]
        self.w.extend(w_vars)
        self.z.extend(z_funcs)
    
    def add_ports(self, inputs, outputs):
        """Add external ports
        
        Args:
            inputs: list of input port variables
            outputs: list of output port variables
        """
        if not isinstance(inputs, list):
            inputs = [inputs]
        if not isinstance(outputs, list):
            outputs = [outputs]
        self.u.extend(inputs)
        self.y.extend(outputs)
    
    def dxH(self):
        """Compute gradient of Hamiltonian"""
        return [sp.diff(self.H, xi) for xi in self.x]
    
    def __add__(self, other):
        """Combine two cores"""
        combined = Core(label=f'{self.label}_{other.label}')
        combined.x = self.x + other.x
        combined.w = self.w + other.w
        combined.z = self.z + other.z
        combined.u = self.u + other.u
        combined.y = self.y + other.y
        combined.H = self.H + other.H
        combined.subs = {**self.subs, **other.subs}
        return combined


class DynamicsCore(Core):
    """
    Core with numerical dynamics capability.

    Each component provides its own dynamics function that computes
    state derivatives given current states and port values.
    
    Attributes:
        n_states: Number of states (set by component builder)
        output_fn: Function to compute outputs from states (set by component builder)
        component_type: Type identifier (e.g., "generator", "exciter")
        model_name: Model name (e.g., "GENROU", "ESST3A")
    """

    def __init__(self, label='system', dynamics_fn=None):
        """
        Args:
            label: Component label
            dynamics_fn: Function with signature (x, ports, metadata) -> x_dot
                        where x is state vector, ports is dict of port values,
                        metadata is dict of parameters
        """
        super().__init__(label)
        self._dynamics_fn = dynamics_fn
        self._metadata = {}
        
        # Component interface attributes (set by builder functions)
        self.n_states = 0
        self.output_fn = None
        self.init_fn = None  # Function to compute equilibrium initial states
        self.component_type = None
        self.model_name = None

    def set_dynamics(self, dynamics_fn):
        """Set the dynamics function after construction"""
        self._dynamics_fn = dynamics_fn

    def set_metadata(self, metadata):
        """Set metadata for dynamics computation"""
        self._metadata = metadata

    def dynamics(self, x, ports):
        """
        Compute state derivatives.

        Args:
            x: numpy array of state values
            ports: dict of port values (e.g., {'Vd': 0.5, 'Vq': 0.8, 'Id': 1.0, 'Iq': 0.5})

        Returns:
            x_dot: numpy array of state derivatives
        """
        if self._dynamics_fn is None:
            raise NotImplementedError(
                f"Dynamics function not implemented for {self.label}. "
                "Provide dynamics_fn in constructor or call set_dynamics()."
            )
        return self._dynamics_fn(x, ports, self._metadata)

    def n_states(self):
        """Return number of states"""
        return len(self.x)
