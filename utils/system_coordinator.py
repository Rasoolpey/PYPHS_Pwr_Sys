"""
System Coordinator - Manages assembled power system with network solver
Dynamically builds network from JSON data without hardcoded connections
Uses proper Kron reduction for network solution
"""
import numpy as np


class PowerSystemCoordinator:
    """Coordinates complete assembled power system"""

    def __init__(self, builder):
        """
        Args:
            builder: PowerSystemBuilder instance with built components
        """
        self.builder = builder
        self.system_data = builder.system_data
        self.n_gen = len(builder.generators)

        # Build mappings from JSON data
        self._build_bus_mapping()
        self._build_generator_mapping()
        self._build_load_mapping()

        # Build network infrastructure from Line data
        self._build_ybus()

        # Build reduced network for generator buses
        self._build_reduced_network()

    def _build_bus_mapping(self):
        """Build bus index mappings from Bus data"""
        buses = self.system_data.get('Bus', [])

        # Map bus idx to internal index (0-based)
        self.bus_idx_to_internal = {}
        self.internal_to_bus_idx = {}

        for i, bus in enumerate(sorted(buses, key=lambda b: b['idx'])):
            self.bus_idx_to_internal[bus['idx']] = i
            self.internal_to_bus_idx[i] = bus['idx']

        self.n_bus = len(buses)
        self.bus_data = {bus['idx']: bus for bus in buses}

    def _build_generator_mapping(self):
        """Build generator to bus mapping from GENROU data"""
        generators = self.system_data.get('GENROU', [])

        # Map generator index to bus
        self.gen_to_bus = {}
        self.bus_to_gens = {}

        for i, gen in enumerate(generators):
            bus_idx = gen['bus']
            self.gen_to_bus[i] = bus_idx

            if bus_idx not in self.bus_to_gens:
                self.bus_to_gens[bus_idx] = []
            self.bus_to_gens[bus_idx].append(i)

        # Get generator buses (internal indices)
        self.gen_bus_internal = [self.bus_idx_to_internal[self.gen_to_bus[i]]
                                  for i in range(self.n_gen)]

        # Unique generator bus indices (some buses may have multiple generators)
        self.unique_gen_buses = sorted(set(self.gen_bus_internal))
        self.n_gen_bus = len(self.unique_gen_buses)

    def _build_load_mapping(self):
        """Build load data from PQ data"""
        pq_loads = self.system_data.get('PQ', [])

        # Initialize load arrays for all buses
        self.load_P = np.zeros(self.n_bus)
        self.load_Q = np.zeros(self.n_bus)

        for load in pq_loads:
            bus_idx = load['bus']
            if bus_idx in self.bus_idx_to_internal:
                internal_idx = self.bus_idx_to_internal[bus_idx]
                self.load_P[internal_idx] += load['p0']
                self.load_Q[internal_idx] += load['q0']

    def _build_ybus(self):
        """Build Ybus matrix dynamically from Line data.

        Note: Per-unit values in JSON are already on common 100 MVA base.
        For transformers, the impedance is given on common base, so NO
        turns ratio scaling is needed in the Ybus construction.
        """
        lines = self.system_data.get('Line', [])

        # Initialize Ybus
        self.Ybus_base = np.zeros((self.n_bus, self.n_bus), dtype=complex)

        # Process each line
        for line in lines:
            bus1_idx = line['bus1']
            bus2_idx = line['bus2']

            # Get internal indices
            if bus1_idx not in self.bus_idx_to_internal or bus2_idx not in self.bus_idx_to_internal:
                continue

            i = self.bus_idx_to_internal[bus1_idx]
            j = self.bus_idx_to_internal[bus2_idx]

            r = line['r']
            x = line['x']
            b = line.get('b', 0.0)

            # Series admittance (per-unit values already on common base)
            z_series = complex(r, x)
            if abs(z_series) > 1e-10:
                y_series = 1.0 / z_series
            else:
                y_series = complex(0, -1e6)  # Very low impedance

            # Shunt admittance (line charging)
            y_shunt = complex(0, b / 2)

            # Standard pi-model for both lines and transformers
            # (per-unit values are already on common 100 MVA base)
            self.Ybus_base[i, i] += y_series + y_shunt
            self.Ybus_base[j, j] += y_series + y_shunt
            self.Ybus_base[i, j] -= y_series
            self.Ybus_base[j, i] -= y_series

    def _build_reduced_network(self):
        """
        Build reduced network using Kron reduction.
        Eliminates non-generator buses to get equivalent Y between generators.
        """
        # Identify generator and non-generator (load) buses
        gen_buses = self.unique_gen_buses
        load_buses = [i for i in range(self.n_bus) if i not in gen_buses]

        n_g = len(gen_buses)
        n_l = len(load_buses)

        if n_l == 0:
            # No load buses, reduced = full
            self.Ybus_reduced = self.Ybus_base[np.ix_(gen_buses, gen_buses)].copy()
            self.gen_bus_to_reduced = {gb: i for i, gb in enumerate(gen_buses)}
            return

        # Partition Ybus: [Ygg Ygl; Ylg Yll]
        Ygg = self.Ybus_base[np.ix_(gen_buses, gen_buses)]
        Ygl = self.Ybus_base[np.ix_(gen_buses, load_buses)]
        Ylg = self.Ybus_base[np.ix_(load_buses, gen_buses)]
        Yll = self.Ybus_base[np.ix_(load_buses, load_buses)]

        # Add load admittances to Yll (constant impedance load model)
        for i, lb in enumerate(load_buses):
            P = self.load_P[lb]
            Q = self.load_Q[lb]
            # Assume nominal voltage = 1.0 for load admittance
            S = P + 1j * Q
            if abs(S) > 1e-6:
                Y_load = np.conj(S)  # Y = S* / |V|^2, assume |V|=1
                Yll[i, i] += Y_load

        # Kron reduction: Y_reduced = Ygg - Ygl * inv(Yll) * Ylg
        try:
            Yll_inv = np.linalg.inv(Yll)
            self.Ybus_reduced = Ygg - Ygl @ Yll_inv @ Ylg
        except np.linalg.LinAlgError:
            # If Yll is singular, use pseudo-inverse
            Yll_inv = np.linalg.pinv(Yll)
            self.Ybus_reduced = Ygg - Ygl @ Yll_inv @ Ylg

        # Mapping from gen bus internal index to reduced matrix index
        self.gen_bus_to_reduced = {gb: i for i, gb in enumerate(gen_buses)}

    def apply_fault(self, bus_idx, impedance):
        """
        Apply fault to network

        Args:
            bus_idx: Bus index (from JSON, will be mapped to internal)
            impedance: Fault impedance (complex)

        Returns:
            Ybus_fault: Modified admittance matrix (full, needs re-reduction)
        """
        Ybus_fault = self.Ybus_base.copy()

        # Map bus_idx to internal index
        if bus_idx in self.bus_idx_to_internal:
            internal_idx = self.bus_idx_to_internal[bus_idx]
            if impedance != 0:
                y_fault = 1.0 / impedance
                Ybus_fault[internal_idx, internal_idx] += y_fault

        return Ybus_fault

    def get_reduced_ybus_with_fault(self, bus_idx, impedance):
        """Get Kron-reduced Ybus with fault applied"""
        Ybus_fault = self.apply_fault(bus_idx, impedance)

        # Re-do Kron reduction with faulted Ybus
        gen_buses = self.unique_gen_buses
        load_buses = [i for i in range(self.n_bus) if i not in gen_buses]

        if len(load_buses) == 0:
            return Ybus_fault[np.ix_(gen_buses, gen_buses)]

        Ygg = Ybus_fault[np.ix_(gen_buses, gen_buses)]
        Ygl = Ybus_fault[np.ix_(gen_buses, load_buses)]
        Ylg = Ybus_fault[np.ix_(load_buses, gen_buses)]
        Yll = Ybus_fault[np.ix_(load_buses, load_buses)]

        # Add load admittances
        for i, lb in enumerate(load_buses):
            P = self.load_P[lb]
            Q = self.load_Q[lb]
            S = P + 1j * Q
            if abs(S) > 1e-6:
                Y_load = np.conj(S)
                Yll[i, i] += Y_load

        try:
            Yll_inv = np.linalg.inv(Yll)
            return Ygg - Ygl @ Yll_inv @ Ylg
        except np.linalg.LinAlgError:
            Yll_inv = np.linalg.pinv(Yll)
            return Ygg - Ygl @ Yll_inv @ Ylg

    def solve_network(self, gen_states, Ybus=None, use_fault=False, fault_bus=None, fault_impedance=None):
        """
        Solve network with Park transformations using FULL network.

        Uses current-balance approach:
        1. Compute E'' from generator states
        2. Inject current at generator buses into full network
        3. Solve for all bus voltages
        4. Extract generator terminal voltages and currents

        Args:
            gen_states: Generator states (n_gen x n_states_per_gen)
            Ybus: If provided, use this Ybus (for backward compatibility)
            use_fault: If True, apply fault
            fault_bus: Bus index for fault
            fault_impedance: Fault impedance

        Returns:
            Id, Iq, Vd, Vq: Currents and voltages in machine frame for each generator
        """
        # Get full Ybus (with or without fault)
        if use_fault and fault_bus is not None:
            Y_full = self.apply_fault(fault_bus, fault_impedance)
        else:
            Y_full = self.Ybus_base.copy()

        # Extract rotor angles
        deltas = gen_states[:, 0]

        # Get generator internal admittances from metadata
        # NOTE: xd2 in metadata is ALREADY on system base (converted in genrou.py)
        y_gen_internal = np.zeros(self.n_gen, dtype=complex)
        for i in range(self.n_gen):
            meta = self.builder.gen_metadata[i]
            Xd2_sys = meta.get('xd2', 0.0278)
            y_gen_internal[i] = 1.0 / (1j * Xd2_sys)

        # Build internal voltages from flux linkages (E'' in machine frame)
        E_internal = np.zeros(self.n_gen, dtype=complex)
        for i in range(self.n_gen):
            meta = self.builder.gen_metadata[i]

            if gen_states.shape[1] >= 7:
                psi_f = gen_states[i, 4] if gen_states.shape[1] > 4 else 1.0
                psi_kd = gen_states[i, 5] if gen_states.shape[1] > 5 else 0
                psi_kq = gen_states[i, 6] if gen_states.shape[1] > 6 else 0

                xd2 = meta.get('xd2', 0.25)
                xd1 = meta.get('xd1', 0.3)
                xl = meta.get('xl', 0.06)
                xq2 = meta.get('xq2', 0.25)
                xq1 = meta.get('xq1', 0.55)

                # E''_q = (xd'' - xl)/(xd' - xl) * psi_f
                if (xd1 - xl) > 1e-6:
                    Eq2 = (xd2 - xl) / (xd1 - xl) * psi_f
                else:
                    Eq2 = psi_f

                # E''_d from q-axis damper
                if (xq1 - xl) > 1e-6:
                    Ed2 = -(xq2 - xl) / (xq1 - xl) * psi_kq
                else:
                    Ed2 = 0.0

                E_internal[i] = Ed2 + 1j * Eq2
            else:
                E_internal[i] = 0.0 + 1j * 1.0

        # Transform internal voltages to system frame (dq -> RI)
        E_sys = np.zeros(self.n_gen, dtype=complex)
        for i in range(self.n_gen):
            e_d = np.real(E_internal[i])
            e_q = np.imag(E_internal[i])
            delta = deltas[i]
            E_sys[i] = (e_d * np.sin(delta) + e_q * np.cos(delta)) + \
                      1j * (-e_d * np.cos(delta) + e_q * np.sin(delta))

        # Build augmented Y matrix: Y_full + generator admittances on diagonal
        Y_aug = Y_full.copy()
        for i in range(self.n_gen):
            gen_bus = self.gen_bus_internal[i]
            Y_aug[gen_bus, gen_bus] += y_gen_internal[i]

        # Current injection from generators: I_inj = Y_gen * E
        I_inj = np.zeros(self.n_bus, dtype=complex)
        for i in range(self.n_gen):
            gen_bus = self.gen_bus_internal[i]
            I_inj[gen_bus] += y_gen_internal[i] * E_sys[i]

        # Iterative solution with constant power loads
        # Start with flat voltage profile
        V_bus = np.ones(self.n_bus, dtype=complex)

        for iteration in range(10):
            # Calculate load current injection (constant power model)
            # I_load = (S/V)* = S* / V*
            I_load = np.zeros(self.n_bus, dtype=complex)
            for lb in range(self.n_bus):
                P = self.load_P[lb]
                Q = self.load_Q[lb]
                if abs(P) > 1e-6 or abs(Q) > 1e-6:
                    S_load = P + 1j * Q
                    if abs(V_bus[lb]) > 0.1:
                        I_load[lb] = np.conj(S_load / V_bus[lb])

            # Total current: generator injection minus load
            I_total = I_inj - I_load

            # Solve for bus voltages
            try:
                V_bus_new = np.linalg.solve(Y_aug, I_total)
            except np.linalg.LinAlgError:
                break

            # Check convergence
            if np.max(np.abs(V_bus_new - V_bus)) < 1e-6:
                V_bus = V_bus_new
                break
            V_bus = V_bus_new

        # Get terminal voltage for each generator
        V_term = np.zeros(self.n_gen, dtype=complex)
        for i in range(self.n_gen):
            gen_bus = self.gen_bus_internal[i]
            V_term[i] = V_bus[gen_bus]

        # Calculate generator output current: I = Y_gen * (E - V)
        I_out = y_gen_internal * (E_sys - V_term)

        # Inverse Park transformation (system to machine frame)
        Vd = np.zeros(self.n_gen)
        Vq = np.zeros(self.n_gen)
        Id = np.zeros(self.n_gen)
        Iq = np.zeros(self.n_gen)

        for i in range(self.n_gen):
            delta = deltas[i]
            v_r = np.real(V_term[i])
            v_i = np.imag(V_term[i])
            i_r = np.real(I_out[i])
            i_i = np.imag(I_out[i])

            # Inverse Park (system to machine)
            Vd[i] = v_r * np.sin(delta) - v_i * np.cos(delta)
            Vq[i] = v_r * np.cos(delta) + v_i * np.sin(delta)
            Id[i] = i_r * np.sin(delta) - i_i * np.cos(delta)
            Iq[i] = i_r * np.cos(delta) + i_i * np.sin(delta)

        return Id, Iq, Vd, Vq

    def get_bus_voltages(self, gen_states):
        """Get voltages at generator buses"""
        Id, Iq, Vd, Vq = self.solve_network(gen_states)

        V_mag = np.sqrt(Vd**2 + Vq**2)
        V_ang = np.arctan2(Vq, Vd) + gen_states[:, 0]

        return V_mag, V_ang

    def summary(self):
        """Print network summary"""
        print("\n=== Network Summary ===")
        print(f"Total buses: {self.n_bus}")
        print(f"Generator buses: {self.n_gen_bus}")
        print(f"Generators: {self.n_gen}")
        print(f"Reduced network size: {self.n_gen_bus}x{self.n_gen_bus}")
        print(f"Total load P: {np.sum(self.load_P):.2f} pu")
        print(f"Total load Q: {np.sum(self.load_Q):.2f} pu")
        print("========================\n")
