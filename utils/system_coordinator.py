"""
System Coordinator - Manages assembled power system with network solver
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
        self.n_gen = len(builder.generators)
        
        # Build network infrastructure
        self._build_ybus()
        
    def _build_ybus(self):
        """Build Ybus matrices from network metadata"""
        meta = self.builder.net_metadata
        X_intra = meta['X_intra']
        X_tie = meta['X_tie']
        
        y_intra = 1.0 / (1j * X_intra)
        y_tie = 1.0 / (1j * X_tie)
        
        # Base Ybus (4x4 for Kundur)
        self.Ybus_base = np.zeros((4, 4), dtype=complex)
        self.Ybus_base[0, 0] = y_intra + y_tie
        self.Ybus_base[0, 1] = -y_intra
        self.Ybus_base[0, 2] = -y_tie
        self.Ybus_base[1, 1] = y_intra
        self.Ybus_base[1, 0] = -y_intra
        self.Ybus_base[2, 2] = y_intra + y_tie
        self.Ybus_base[2, 3] = -y_intra
        self.Ybus_base[2, 0] = -y_tie
        self.Ybus_base[3, 3] = y_intra
        self.Ybus_base[3, 2] = -y_intra
        
        # Loads
        self.load_P = np.array([11.59/2, 11.59/2, 15.75/2, 15.75/2])
        self.load_Q = np.array([-0.735/2, -0.735/2, -0.899/2, -0.899/2])
    
    def apply_fault(self, bus_idx, impedance):
        """
        Apply fault to network
        
        Args:
            bus_idx: Bus index (0-3 for Kundur)
            impedance: Fault impedance (complex)
        
        Returns:
            Ybus_fault: Modified admittance matrix
        """
        Ybus_fault = self.Ybus_base.copy()
        if impedance != 0:
            y_fault = 1.0 / impedance
            Ybus_fault[bus_idx, bus_idx] += y_fault
        return Ybus_fault
    
    def solve_network(self, gen_states, Ybus=None):
        """
        Solve network with Park transformations
        
        Args:
            gen_states: Generator states (n_gen x 7)
            Ybus: Admittance matrix (if None, use base)
        
        Returns:
            Id, Iq, Vd, Vq: Currents and voltages in machine frame
        """
        if Ybus is None:
            Ybus = self.Ybus_base
        
        deltas = gen_states[:, 0]
        
        # Gen admittance
        Xd2 = 0.25 * (100.0 / 900.0)
        y_gen = 1.0 / (1j * Xd2)
        Y_gen = np.eye(4) * y_gen
        
        # Internal voltages
        E_internal = np.zeros(4, dtype=complex)
        for i in range(4):
            meta = self.builder.gen_metadata[i]
            psi_f = gen_states[i, 4]
            psi_kd = gen_states[i, 5]
            psi_kq = gen_states[i, 6]
            
            psi2d = meta['gd1'] * psi_f + meta['gd2'] * (meta['xd1'] - meta['xl']) * psi_kd
            psi2q = (1 - meta['gq1']) * psi_kq
            E_internal[i] = psi2d + 1j * psi2q
        
        # Forward Park
        E_sys = np.zeros(4, dtype=complex)
        for i in range(4):
            e_d, e_q = np.real(E_internal[i]), np.imag(E_internal[i])
            delta = deltas[i]
            E_sys[i] = (e_d * np.cos(delta) - e_q * np.sin(delta)) + \
                      1j * (e_d * np.sin(delta) + e_q * np.cos(delta))
        
        # Network solve
        I_gen = E_sys * y_gen
        S_load = self.load_P + 1j * self.load_Q
        V_est = np.ones(4, dtype=complex)
        I_load = np.conj(S_load / V_est)
        
        I_inj = I_gen - I_load
        Y_total = Ybus + Y_gen
        V_term = np.linalg.solve(Y_total, I_inj)
        
        # Refine
        V_safe = np.where(np.abs(V_term) > 0.5, V_term, np.ones(4, dtype=complex))
        I_load = np.conj(S_load / V_safe)
        I_inj = I_gen - I_load
        V_term = np.linalg.solve(Y_total, I_inj)
        
        I_out = (E_sys - V_term) * y_gen
        
        # Inverse Park
        Vd = np.zeros(4)
        Vq = np.zeros(4)
        Id = np.zeros(4)
        Iq = np.zeros(4)
        
        for i in range(4):
            delta = deltas[i]
            v_r, v_i = np.real(V_term[i]), np.imag(V_term[i])
            i_r, i_i = np.real(I_out[i]), np.imag(I_out[i])
            
            Vd[i] = v_r * np.cos(delta) + v_i * np.sin(delta)
            Vq[i] = -v_r * np.sin(delta) + v_i * np.cos(delta)
            Id[i] = i_r * np.cos(delta) + i_i * np.sin(delta)
            Iq[i] = -i_r * np.sin(delta) + i_i * np.cos(delta)
        
        return Id, Iq, Vd, Vq
