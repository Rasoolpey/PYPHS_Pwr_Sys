# """
# IMTB Scanner - MIMO Impedance Scanning for Port-Hamiltonian Systems
# Updated: "Natural Settling" initialization to find true rotor angles.
# """
# import numpy as np
# import scipy.signal as signal
# from scipy.integrate import solve_ivp
# from utils.system_builder import PowerSystemBuilder
# from utils.system_coordinator import PowerSystemCoordinator
# import utils.im_analysis_lib as IM

# class IMTBScanner:
#     def __init__(self, json_file):
#         self.builder = PowerSystemBuilder(json_file)
#         self.builder.build_all_components()
#         self.coordinator = PowerSystemCoordinator(self.builder)
        
#         self.n_gen = len(self.builder.generators)
#         self.states_per_machine = 13
#         self.total_states = self.n_gen * self.states_per_machine
        
#         # Settings
#         self.fs = 2000.0  
#         self.T_settle = 2.0
        
#         # Internal Storage
#         self._t_eval = None
#         self._inj_d = None
#         self._inj_q = None
        
#         # Stability State
#         self.stabilize = False
#         self.delta0 = np.zeros(self.n_gen) 

#     def list_buses(self):
#         buses = []
#         for i, meta in enumerate(self.builder.gen_metadata):
#             buses.append({'idx': i, 'name': f"Gen {i+1} (Bus {meta['bus']})", 'bus_id': meta['bus']})
#         return buses

#     def _extract_param(self, core, param_name, default=1.0):
#         for k, v in core.subs.items():
#             if param_name in str(k): return float(v)
#         return default

#     def _solve_network_with_injection(self, gen_states, target_bus, Id_inj, Iq_inj):
#         Ybus = self.coordinator.Ybus_base
        
#         Y_gen_diag = np.zeros(self.n_gen, dtype=complex)
#         for i in range(self.n_gen):
#             xd2_pu = self.builder.gen_metadata[i].get('xd2', 0.25)
#             Y_gen_diag[i] = 1.0 / (1j * xd2_pu)

#         E_sys = np.zeros(self.n_gen, dtype=complex)
#         for i in range(self.n_gen):
#             meta = self.builder.gen_metadata[i]
#             delta, _, _, _, psi_f, psi_kd, psi_kq = gen_states[i]
#             psi2d = meta['gd1'] * psi_f + meta['gd2'] * (meta['xd1'] - meta['xl']) * psi_kd
#             psi2q = (1 - meta['gq1']) * psi_kq
#             E_int = psi2d + 1j * psi2q
#             E_sys[i] = (np.real(E_int) * np.cos(delta) - np.imag(E_int) * np.sin(delta)) + \
#                        1j * (np.real(E_int) * np.sin(delta) + np.imag(E_int) * np.cos(delta))

#         I_gen_source = E_sys * Y_gen_diag
#         S_load = self.coordinator.load_P + 1j * self.coordinator.load_Q
#         V_term = np.ones(self.n_gen, dtype=complex)
#         Y_total = Ybus + np.diag(Y_gen_diag)
        
#         I_inj_net_complex = 0j
#         if target_bus >= 0:
#             delta_target = gen_states[target_bus, 0]
#             I_inj_net_complex = (Id_inj * np.cos(delta_target) - Iq_inj * np.sin(delta_target)) + \
#                                 1j * (Id_inj * np.sin(delta_target) + Iq_inj * np.cos(delta_target))

#         for _ in range(3):
#             I_load = np.conj(S_load / V_term)
#             I_rhs = I_gen_source - I_load
#             if target_bus >= 0:
#                 I_rhs[target_bus] += I_inj_net_complex
#             V_term = np.linalg.solve(Y_total, I_rhs)
            
#         I_out = (E_sys - V_term) * Y_gen_diag
        
#         Vd = np.zeros(self.n_gen); Vq = np.zeros(self.n_gen)
#         Id = np.zeros(self.n_gen); Iq = np.zeros(self.n_gen)
#         for i in range(self.n_gen):
#             delta = gen_states[i, 0]
#             v_r, v_i = np.real(V_term[i]), np.imag(V_term[i])
#             i_r, i_i = np.real(I_out[i]), np.imag(I_out[i])
#             Vd[i] = v_r * np.cos(delta) + v_i * np.sin(delta)
#             Vq[i] = -v_r * np.sin(delta) + v_i * np.cos(delta)
#             Id[i] = i_r * np.cos(delta) + i_i * np.sin(delta)
#             Iq[i] = -i_r * np.sin(delta) + i_i * np.cos(delta)
            
#         return Vd, Vq, Id, Iq

#     def _dynamics(self, t, x_flat, target_bus):
#         x = x_flat.reshape(self.n_gen, self.states_per_machine)
#         gen_states = x[:, :7]
        
#         id_inj = 0.0; iq_inj = 0.0
#         if target_bus >= 0 and self._t_eval is not None:
#             id_inj = np.interp(t, self._t_eval, self._inj_d)
#             iq_inj = np.interp(t, self._t_eval, self._inj_q)
            
#         Vd, Vq, Id, Iq = self._solve_network_with_injection(gen_states, target_bus, id_inj, iq_inj)
        
#         dxdt = np.zeros_like(x)
#         for i in range(self.n_gen):
#             meta = self.builder.gen_metadata[i]
#             gen_core = self.builder.generators[i]
#             exc_core = self.builder.exciters[i]
#             gov_core = self.builder.governors[i]
            
#             delta, p, psi_d, psi_q, psi_f, psi_kd, psi_kq = gen_states[i]
#             vm, vr1, vr2, vf = x[i, 7:11]
#             x1, x2 = x[i, 11:13]
#             omega = p / meta['M']
            
#             gov_m = self.builder.gov_metadata[i]
#             exc_m = self.builder.exc_metadata[i]
            
#             Td10 = self._extract_param(gen_core, 'Td10', 8.0)
#             Td20 = self._extract_param(gen_core, 'Td20', 0.03)
#             Tq20 = self._extract_param(gen_core, 'Tq20', 0.05)
#             TR = self._extract_param(exc_core, 'TR', 0.02)
#             TA = self._extract_param(exc_core, 'TA', 0.02)
#             KA = self._extract_param(exc_core, 'KA', 200.0)
            
#             Tm = (gov_m['T2']/gov_m['T3'])*(x1 - x2) + x2 - gov_m['Dt']*(omega-1.0)
#             Te = Vd[i]*Id[i] + Vq[i]*Iq[i]
            
#             # --- STABILITY LOGIC ---
#             # If stabilize=True, we use a Spring (K) + Damper (D) to lock the angle.
#             # If stabilize=False (during settling), we use ONLY Damper (D) to let it slide.
#             T_stabilizer = 0.0
            
#             if self.stabilize:
#                 # LOCKED MODE: Hold current angle
#                 K_spring = 1e6   
#                 D_damper = 1e4   
#                 delta_err = delta - self.delta0[i]
#                 speed_err = omega - 1.0
#                 T_stabilizer = K_spring * delta_err + D_damper * speed_err
#             else:
#                 # SLIDING MODE: Just high friction to find equilibrium
#                 # No Spring (K=0), just Damper
#                 D_damper = 100.0
#                 speed_err = omega - 1.0
#                 T_stabilizer = D_damper * speed_err
            
#             dxdt[i, 0] = meta['omega_b'] * (omega - 1.0)
#             dxdt[i, 1] = Tm - Te - meta['D']*(omega - 1.0) - T_stabilizer
#             dxdt[i, 2] = Vd[i] - meta['ra']*Id[i] + meta['omega_b']*omega*psi_q
#             dxdt[i, 3] = Vq[i] - meta['ra']*Iq[i] - meta['omega_b']*omega*psi_d
            
#             Efd = vr1
#             dxdt[i, 4] = (Efd - psi_f) / Td10
#             dxdt[i, 5] = -psi_kd / Td20
#             dxdt[i, 6] = -psi_kq / Tq20
            
#             Vt = np.sqrt(Vd[i]**2 + Vq[i]**2)
#             dxdt[i, 7] = (Vt - vm) / TR
#             Vref = exc_m.get('Vref_setpoint', 1.0)
#             Verr = Vref - vm
#             dxdt[i, 8] = (KA * Verr - vr1) / TA
#             dxdt[i, 9] = 0.0; dxdt[i, 10] = 0.0
            
#             Pref = gov_m.get('Pref_setpoint', gov_m['Pref'])
#             gate = Pref + (gov_m['wref'] - omega)/gov_m['R']
#             dxdt[i, 11] = (gate - x1) * 10.0 / gov_m['T1']
#             dxdt[i, 12] = (x1 - x2) * 10.0 / gov_m['T3']
            
#         return dxdt.flatten()

#     def initialize_equilibrium(self):
#         """Iterative Trim with Natural Settling"""
#         print("Finding equilibrium (Iterative Trim)...")
        
#         # 1. Flat Start
#         x0_flat = np.zeros(self.total_states)
#         for i in range(self.n_gen):
#             x0_flat[i*13 + 0] = 0.0; x0_flat[i*13 + 1] = self.builder.gen_metadata[i]['M']
#             x0_flat[i*13 + 4] = 1.0; x0_flat[i*13 + 7] = 1.0
#             x0_flat[i*13 + 11] = 7.0; x0_flat[i*13 + 12] = 7.0

#         self._t_eval = None
#         self.delta0 = np.zeros(self.n_gen) 
        
#         x_current = x0_flat
        
#         # --- ITERATION LOOP ---
#         for iteration in range(3):
#             print(f"  Trim Iteration {iteration+1} (Settling for 20s)...")
            
#             # STEP A: SLIDING MODE (Stabilize=False, High Damping)
#             # Allows rotors to move to natural load angles
#             self.stabilize = False 
            
#             # Run for 20 seconds to let angles drift
#             sol = solve_ivp(lambda t,x: self._dynamics(t,x,-1), (0, 20.0), x_current, method='RK45', max_step=0.1)
#             x_current = sol.y[:, -1]
            
#             # STEP B: UPDATE REFERENCES
#             x_reshaped = x_current.reshape(self.n_gen, self.states_per_machine)
#             gen_states = x_reshaped[:, :7]
#             Vd, Vq, Id, Iq = self._solve_network_with_injection(gen_states, -1, 0, 0)
            
#             for i in range(self.n_gen):
#                 Vt_eq = np.sqrt(Vd[i]**2 + Vq[i]**2)
#                 self.builder.exc_metadata[i]['Vref_setpoint'] = Vt_eq
                
#                 Pe_calc = Vd[i]*Id[i] + Vq[i]*Iq[i]
#                 self.builder.gov_metadata[i]['Pref_setpoint'] = Pe_calc
                
#                 # Capture the NATURAL angle
#                 self.delta0[i] = x_reshaped[i, 0]
                
#             print(f"    Gen 1: Pref -> {self.builder.gov_metadata[0]['Pref_setpoint']:.4f}, Angle -> {np.degrees(self.delta0[0]):.2f} deg")

#         print(f"  Final Locked Angles: {np.degrees(self.delta0)}")
#         return x_current

#     def generate_multisine(self, duration, freqs, amplitude):
#         N = int(duration * self.fs)
#         t = np.linspace(0, duration, N)
#         sig = np.zeros(N)
#         phases = np.random.uniform(0, 2*np.pi, len(freqs))
#         for i, f in enumerate(freqs):
#             sig += np.sin(2 * np.pi * f * t + phases[i])
#         sig = sig / np.max(np.abs(sig)) * amplitude
#         return t, sig

#     def run_mimo_scan(self, bus_idx, freqs_hz, amplitude=0.01, duration=5.0):
#         print(f"Starting MIMO Scan on Bus {bus_idx}...")
        
#         x_start = self.initialize_equilibrium()
        
#         # ENABLE LOCK for Scan
#         self.stabilize = True 
        
#         t_inj, sig_inj = self.generate_multisine(duration, freqs_hz, amplitude)
#         zeros = np.zeros_like(sig_inj)
        
#         print(f"  Exp 1: Injecting Id (Amp={amplitude} pu)...")
#         self._t_eval = t_inj; self._inj_d = sig_inj; self._inj_q = zeros
#         sol1 = solve_ivp(lambda t,x: self._dynamics(t,x,bus_idx), (0, duration), x_start, 
#                          t_eval=t_inj, method='RK45', max_step=1e-3)

#         print(f"  Exp 2: Injecting Iq (Amp={amplitude} pu)...")
#         self._inj_d = zeros; self._inj_q = sig_inj
#         sol2 = solve_ivp(lambda t,x: self._dynamics(t,x,bus_idx), (0, duration), x_start, 
#                          t_eval=t_inj, method='RK45', max_step=1e-3)
        
#         print("  Calculating Impedance Matrix...")
#         Z_mimo = np.zeros((len(freqs_hz), 2, 2), dtype=complex)
        
#         def recover_signals_dynamic(sol, inj_d_arr, inj_q_arr):
#             N = len(sol.t)
#             Vd_rec = np.zeros(N); Vq_rec = np.zeros(N)
#             Id_rec = np.zeros(N); Iq_rec = np.zeros(N)
#             for k in range(N):
#                 x_k = sol.y[:, k].reshape(self.n_gen, 13)
#                 gen_st = x_k[:, :7]
#                 i_d = inj_d_arr[k]; i_q = inj_q_arr[k]
#                 vd, vq, _, _ = self._solve_network_with_injection(gen_st, bus_idx, i_d, i_q)
#                 Vd_rec[k] = vd[bus_idx]; Vq_rec[k] = vq[bus_idx]
#                 Id_rec[k] = i_d; Iq_rec[k] = i_q
#             return Vd_rec, Vq_rec, Id_rec, Iq_rec

#         Vd1, Vq1, Id1, Iq1 = recover_signals_dynamic(sol1, sig_inj, zeros)
#         Vd2, Vq2, Id2, Iq2 = recover_signals_dynamic(sol2, zeros, sig_inj)
        
#         for i, f in enumerate(freqs_hz):
#             vd1_f = IM.DFT_1f(Vd1, self.fs, f); vq1_f = IM.DFT_1f(Vq1, self.fs, f)
#             id1_f = IM.DFT_1f(Id1, self.fs, f); iq1_f = IM.DFT_1f(Iq1, self.fs, f)
#             vd2_f = IM.DFT_1f(Vd2, self.fs, f); vq2_f = IM.DFT_1f(Vq2, self.fs, f)
#             id2_f = IM.DFT_1f(Id2, self.fs, f); iq2_f = IM.DFT_1f(Iq2, self.fs, f)
#             Z_mat = IM.calc_MIMO_Z(vd1_f, vq1_f, id1_f, iq1_f, vd2_f, vq2_f, id2_f, iq2_f)
#             Z_mimo[i, :, :] = Z_mat
            
#         return freqs_hz, Z_mimo

"""
IMTB Scanner - MIMO Impedance Scanning for Port-Hamiltonian Systems
Fixed: Proper equilibrium initialization + DC bias removal
"""
import numpy as np
from scipy.integrate import solve_ivp
from utils.system_builder import PowerSystemBuilder
from utils.system_coordinator import PowerSystemCoordinator
import utils.im_analysis_lib as IM

class IMTBScanner:
    def __init__(self, json_file):
        self.builder = PowerSystemBuilder(json_file)
        self.builder.build_all_components()
        self.coordinator = PowerSystemCoordinator(self.builder)
        
        self.n_gen = len(self.builder.generators)
        self.states_per_machine = 13
        self.total_states = self.n_gen * self.states_per_machine
        
        self.fs = 2000.0  
        self.T_settle = 2.0
        
        self._t_eval = None
        self._inj_d = None
        self._inj_q = None
        
        # Equilibrium storage
        self.x_eq = None
        self.delta_eq = None

    def list_buses(self):
        buses = []
        for i, meta in enumerate(self.builder.gen_metadata):
            buses.append({
                'idx': i,
                'name': f"Gen {i+1} (Bus {meta['bus']})",
                'bus_id': meta['bus']
            })
        return buses

    def _extract_param(self, core, param_name, default=1.0):
        for k, v in core.subs.items():
            if param_name in str(k):
                return float(v)
        return default

    def _solve_network_with_injection(self, gen_states, target_bus, Id_inj, Iq_inj):
        Ybus = self.coordinator.Ybus_base
        
        # CRITICAL FIX: Use same xd2 calculation as SystemCoordinator
        # xd2 should be scaled by base ratio
        Y_gen_diag = np.zeros(self.n_gen, dtype=complex)
        for i in range(self.n_gen):
            xd2_base = 0.25  # Machine base
            Sn = self.builder.gen_metadata[i].get('Sn', 900.0)
            S_system = 100.0
            xd2_pu = xd2_base * (S_system / Sn)  # System pu
            Y_gen_diag[i] = 1.0 / (1j * xd2_pu)

        E_sys = np.zeros(self.n_gen, dtype=complex)
        for i in range(self.n_gen):
            meta = self.builder.gen_metadata[i]
            delta, _, _, _, psi_f, psi_kd, psi_kq = gen_states[i]
            psi2d = meta['gd1'] * psi_f + meta['gd2'] * (meta['xd1'] - meta['xl']) * psi_kd
            psi2q = (1 - meta['gq1']) * psi_kq
            E_int = psi2d + 1j * psi2q
            E_sys[i] = (np.real(E_int) * np.cos(delta) - np.imag(E_int) * np.sin(delta)) + \
                       1j * (np.real(E_int) * np.sin(delta) + np.imag(E_int) * np.cos(delta))

        I_gen_source = E_sys * Y_gen_diag
        S_load = self.coordinator.load_P + 1j * self.coordinator.load_Q
        V_term = np.ones(self.n_gen, dtype=complex)
        Y_total = Ybus + np.diag(Y_gen_diag)
        
        I_inj_net_complex = 0j
        if target_bus >= 0:
            delta_target = gen_states[target_bus, 0]
            I_inj_net_complex = (Id_inj * np.cos(delta_target) - Iq_inj * np.sin(delta_target)) + \
                                1j * (Id_inj * np.sin(delta_target) + Iq_inj * np.cos(delta_target))

        # First iteration
        I_load = np.conj(S_load / V_term)
        I_rhs = I_gen_source - I_load
        if target_bus >= 0:
            I_rhs[target_bus] += I_inj_net_complex
        V_term = np.linalg.solve(Y_total, I_rhs)
        
        # Refine (2nd iteration) - match SystemCoordinator
        V_safe = np.where(np.abs(V_term) > 0.5, V_term, np.ones(self.n_gen, dtype=complex))
        I_load = np.conj(S_load / V_safe)
        I_rhs = I_gen_source - I_load
        if target_bus >= 0:
            I_rhs[target_bus] += I_inj_net_complex
        V_term = np.linalg.solve(Y_total, I_rhs)
            
        I_out = (E_sys - V_term) * Y_gen_diag
        
        Vd = np.zeros(self.n_gen); Vq = np.zeros(self.n_gen)
        Id = np.zeros(self.n_gen); Iq = np.zeros(self.n_gen)
        for i in range(self.n_gen):
            delta = gen_states[i, 0]
            v_r, v_i = np.real(V_term[i]), np.imag(V_term[i])
            i_r, i_i = np.real(I_out[i]), np.imag(I_out[i])
            Vd[i] = v_r * np.cos(delta) + v_i * np.sin(delta)
            Vq[i] = -v_r * np.sin(delta) + v_i * np.cos(delta)
            Id[i] = i_r * np.cos(delta) + i_i * np.sin(delta)
            Iq[i] = -i_r * np.sin(delta) + i_i * np.cos(delta)
            
        return Vd, Vq, Id, Iq

    def _dynamics(self, t, x_flat, target_bus):
        x = x_flat.reshape(self.n_gen, self.states_per_machine)
        gen_states = x[:, :7]
        
        id_inj = 0.0; iq_inj = 0.0
        if target_bus >= 0 and self._t_eval is not None:
            id_inj = np.interp(t, self._t_eval, self._inj_d)
            iq_inj = np.interp(t, self._t_eval, self._inj_q)
            
        Vd, Vq, Id, Iq = self._solve_network_with_injection(gen_states, target_bus, id_inj, iq_inj)
        
        dxdt = np.zeros_like(x)
        for i in range(self.n_gen):
            meta = self.builder.gen_metadata[i]
            gen_core = self.builder.generators[i]
            exc_core = self.builder.exciters[i]
            gov_core = self.builder.governors[i]
            
            delta, p, psi_d, psi_q, psi_f, psi_kd, psi_kq = gen_states[i]
            vm, vr1, vr2, vf = x[i, 7:11]
            x1, x2 = x[i, 11:13]
            omega = p / meta['M']
            
            gov_m = self.builder.gov_metadata[i]
            exc_m = self.builder.exc_metadata[i]
            
            Td10 = self._extract_param(gen_core, 'Td10', 8.0)
            Td20 = self._extract_param(gen_core, 'Td20', 0.03)
            Tq20 = self._extract_param(gen_core, 'Tq20', 0.05)
            TR = self._extract_param(exc_core, 'TR', 0.02)
            TA = self._extract_param(exc_core, 'TA', 0.02)
            KA = self._extract_param(exc_core, 'KA', 200.0)
            
            Tm = (gov_m['T2']/gov_m['T3'])*(x1 - x2) + x2 - gov_m['Dt']*(omega-1.0)
            Te = Vd[i]*Id[i] + Vq[i]*Iq[i]
            
            dxdt[i, 0] = meta['omega_b'] * (omega - 1.0)
            dxdt[i, 1] = Tm - Te - meta['D']*(omega - 1.0)
            dxdt[i, 2] = Vd[i] - meta['ra']*Id[i] + meta['omega_b']*omega*psi_q
            dxdt[i, 3] = Vq[i] - meta['ra']*Iq[i] - meta['omega_b']*omega*psi_d
            
            Efd = vr1
            dxdt[i, 4] = (Efd - psi_f) / Td10
            dxdt[i, 5] = -psi_kd / Td20
            dxdt[i, 6] = -psi_kq / Tq20
            
            Vt = np.sqrt(Vd[i]**2 + Vq[i]**2)
            dxdt[i, 7] = (Vt - vm) / TR
            
            # Vref from exciter core
            Vref = self._extract_param(exc_core, 'Vref', 1.0)
            Verr = Vref - vm
            
            # CRITICAL: Add exciter saturation (same as fault_sim_modular)
            exc_m = self.builder.exc_metadata[i]
            VRMAX = exc_m.get('VRMAX', 5.2)
            VRMIN = exc_m.get('VRMIN', -4.16)
            
            vr1_unlimited = (KA * Verr - vr1) / TA
            
            # Anti-windup saturation
            if vr1 >= VRMAX and vr1_unlimited > 0:
                dxdt[i, 8] = 0.0
            elif vr1 <= VRMIN and vr1_unlimited < 0:
                dxdt[i, 8] = 0.0
            else:
                dxdt[i, 8] = vr1_unlimited
            
            dxdt[i, 9] = 0.0
            dxdt[i, 10] = 0.0
            
            # Governor - use Pref from metadata (updated in initialization)
            gate_cmd = gov_m['Pref'] + (gov_m['wref'] - omega) / gov_m['R']
            gate_limited = np.clip(gate_cmd, gov_m.get('VMIN', 0.0), gov_m.get('VMAX', 10.0))
            dxdt[i, 11] = 10.0 * (gate_limited - x1) / gov_m['T1']
            dxdt[i, 12] = 10.0 * (x1 - x2) / gov_m['T3']
            
        return dxdt.flatten()

    def _check_stability(self, x_flat):
        """Check if system state is reasonable"""
        x = x_flat.reshape(self.n_gen, 13)
        deltas = x[:, 0]
        speeds = x[:, 1] / np.array([m['M'] for m in self.builder.gen_metadata])
        
        delta_range = np.max(deltas) - np.min(deltas)
        speed_dev = np.max(np.abs(speeds - 1.0))
        
        stable = (delta_range < np.deg2rad(60)) and (speed_dev < 0.05)
        return stable, delta_range, speed_dev

    def initialize_equilibrium(self):
        """Find equilibrium - identical to fault_sim_modular"""
        print("Finding equilibrium...")
        
        # Initial guesses (from fault_sim_modular)
        P_targets = np.array([7.459, 7.0, 7.0, 7.0])
        delta_init = np.array([0.08, 0.07, 0.06, 0.07])
        psi_f_init = np.array([1.3, 1.3, 1.3, 1.3])
        
        # Iterative trim for voltage and power
        for iteration in range(50):
            gen_states = np.zeros((self.n_gen, 7))
            for i in range(self.n_gen):
                M = self.builder.gen_metadata[i]['M']
                gen_states[i] = [delta_init[i], M * 1.0, 0.0, 0.0, psi_f_init[i], 0.0, 0.0]
            
            Vd, Vq, Id, Iq = self._solve_network_with_injection(gen_states, -1, 0.0, 0.0)
            P_calc = Vd * Id + Vq * Iq
            V_mag = np.sqrt(Vd**2 + Vq**2)
            
            max_error = 0
            for i in range(self.n_gen):
                P_error = P_targets[i] - P_calc[i]
                delta_init[i] += 0.02 * P_error
                
                V_error = 1.0 - V_mag[i]
                psi_f_init[i] += 0.1 * V_error
                psi_f_init[i] = np.clip(psi_f_init[i], 0.5, 2.0)
                
                max_error = max(max_error, abs(P_error))
            
            if iteration % 10 == 0:
                print(f"  Iter {iteration}: P_error={max_error:.5f}")
            
            if max_error < 1e-4:
                print(f"  Converged in {iteration+1} iterations")
                break
        
        # Build full state vector
        x0 = np.zeros((self.n_gen, self.states_per_machine))
        
        for i in range(self.n_gen):
            meta = self.builder.gen_metadata[i]
            omega_b = meta['omega_b']
            M = meta['M']
            ra = meta['ra']
            
            # Flux linkages from voltage equations
            psi_q = (-Vd[i] - ra * Id[i]) / omega_b
            psi_d = (Vq[i] + ra * Iq[i]) / omega_b
            
            # Generator states
            x0[i, 0] = delta_init[i]
            x0[i, 1] = M * 1.0
            x0[i, 2] = psi_d
            x0[i, 3] = psi_q
            x0[i, 4] = psi_f_init[i]
            x0[i, 5] = 0.0
            x0[i, 6] = 0.0
            
            # Exciter states
            x0[i, 7] = V_mag[i]
            x0[i, 8] = psi_f_init[i]  # vr1 = Efd at equilibrium
            x0[i, 9] = 0.0
            x0[i, 10] = 0.0
            
            # Governor states - CRITICAL: use actual calculated power
            P_eq = P_calc[i]  # Use calculated power, not target!
            x0[i, 11] = P_eq
            x0[i, 12] = P_eq
            
            # Update Pref in gov_metadata (this is used by dynamics)
            self.builder.gov_metadata[i]['Pref'] = P_eq
            
            # Update Vref to maintain equilibrium Efd
            # At equilibrium: d(Efd)/dt = 0 => KA*(Vref - Vt) = Efd
            # So: Vref = Vt + Efd/KA
            exc_core = self.builder.exciters[i]
            KA = self._extract_param(exc_core, 'KA', 200.0)
            Efd_eq = x0[i, 8]
            Vref_correct = V_mag[i] + Efd_eq / KA

            for key in exc_core.subs.keys():
                if 'Vref' in str(key):
                    exc_core.subs[key] = Vref_correct
                    break
        
        print("Equilibrium:")
        for i in range(self.n_gen):
            print(f"  Gen {i}: delta={np.degrees(delta_init[i]):.2f} deg, P={P_calc[i]:.3f}, V={V_mag[i]:.4f}")
        
        # Store for diagnostics
        self.x_eq = x0.flatten().copy()
        self.delta_eq = delta_init.copy()
        
        return x0.flatten()

    def generate_multisine(self, duration, freqs, amplitude):
        N = int(duration * self.fs)
        t = np.linspace(0, duration, N)
        sig = np.zeros(N)
        phases = np.random.uniform(0, 2*np.pi, len(freqs))
        for i, f in enumerate(freqs):
            sig += np.sin(2 * np.pi * f * t + phases[i])
        sig = sig / np.max(np.abs(sig)) * amplitude
        return t, sig

    def _remove_dc_bias(self, signal):
        """Remove DC component"""
        return signal - np.mean(signal)

    def run_mimo_scan(self, bus_idx, freqs_hz, amplitude=0.01, duration=5.0):
        print(f"Starting MIMO Scan on Bus {bus_idx}...")
        
        x_start = self.initialize_equilibrium()
        
        t_inj, sig_inj = self.generate_multisine(duration, freqs_hz, amplitude)
        zeros = np.zeros_like(sig_inj)
        
        print(f"  Exp 1: Injecting Id (Amp={amplitude} pu)...")
        self._t_eval = t_inj
        self._inj_d = sig_inj
        self._inj_q = zeros
        sol1 = solve_ivp(lambda t,x: self._dynamics(t,x,bus_idx), (0, duration), x_start, 
                         t_eval=t_inj, method='RK45', max_step=1e-3)
        
        # Check stability
        stable1, dr1, sd1 = self._check_stability(sol1.y[:, -1])
        if not stable1:
            print(f"    WARNING: Exp1 unstable! Δδ={np.degrees(dr1):.1f}° Δω={sd1:.3f}")

        print(f"  Exp 2: Injecting Iq (Amp={amplitude} pu)...")
        self._inj_d = zeros
        self._inj_q = sig_inj
        sol2 = solve_ivp(lambda t,x: self._dynamics(t,x,bus_idx), (0, duration), x_start, 
                         t_eval=t_inj, method='RK45', max_step=1e-3)
        
        stable2, dr2, sd2 = self._check_stability(sol2.y[:, -1])
        if not stable2:
            print(f"    WARNING: Exp2 unstable! Δδ={np.degrees(dr2):.1f}° Δω={sd2:.3f}")
        
        # Drift diagnostics
        def check_drift(sol):
            y = sol.y[:, -1].reshape(self.n_gen, 13)
            y0 = sol.y[:, 0].reshape(self.n_gen, 13)
            drift = np.degrees(y[bus_idx, 0] - y0[bus_idx, 0])
            # Unwrap: if drift > 180, it wrapped around
            if abs(drift) > 180:
                drift = ((drift + 180) % 360) - 180
            return drift
            
        d1, d2 = check_drift(sol1), check_drift(sol2)
        print(f"  > Rotor Drift: Exp1={d1:.2f}°, Exp2={d2:.2f}°")
        
        if abs(d1) > 10 or abs(d2) > 10:
            print("  ! Large drift detected - results may be inaccurate!")

        print("  Calculating Impedance Matrix...")
        Z_mimo = np.zeros((len(freqs_hz), 2, 2), dtype=complex)
        
        def recover_signals_dynamic(sol, inj_d_arr, inj_q_arr):
            N = len(sol.t)
            Vd_rec = np.zeros(N); Vq_rec = np.zeros(N)
            Id_rec = np.zeros(N); Iq_rec = np.zeros(N)
            for k in range(N):
                x_k = sol.y[:, k].reshape(self.n_gen, 13)
                gen_st = x_k[:, :7]
                i_d = inj_d_arr[k]; i_q = inj_q_arr[k]
                vd, vq, _, _ = self._solve_network_with_injection(gen_st, bus_idx, i_d, i_q)
                Vd_rec[k] = vd[bus_idx]; Vq_rec[k] = vq[bus_idx]
                Id_rec[k] = i_d; Iq_rec[k] = i_q
            return Vd_rec, Vq_rec, Id_rec, Iq_rec

        Vd1, Vq1, Id1, Iq1 = recover_signals_dynamic(sol1, sig_inj, zeros)
        Vd2, Vq2, Id2, Iq2 = recover_signals_dynamic(sol2, zeros, sig_inj)
        
        # Diagnostic plots
        print("\n  === SIGNAL DIAGNOSTICS ===")
        print(f"  Bus {bus_idx} - Experiment 1 (Id injection):")
        print(f"    Vd: mean={np.mean(Vd1):.4f}, std={np.std(Vd1):.4f}, range=[{np.min(Vd1):.4f}, {np.max(Vd1):.4f}]")
        print(f"    Vq: mean={np.mean(Vq1):.4f}, std={np.std(Vq1):.4f}, range=[{np.min(Vq1):.4f}, {np.max(Vq1):.4f}]")
        print(f"    Id: mean={np.mean(Id1):.4f}, std={np.std(Id1):.4f}, range=[{np.min(Id1):.4f}, {np.max(Id1):.4f}]")
        print(f"    Iq: mean={np.mean(Iq1):.4f}, std={np.std(Iq1):.4f}, range=[{np.min(Iq1):.4f}, {np.max(Iq1):.4f}]")
        
        # CRITICAL FIX: Transpose before reshape
        x1 = sol1.y.T.reshape(len(sol1.t), self.n_gen, 13)
        omega1 = x1[:, bus_idx, 1] / self.builder.gen_metadata[bus_idx]['M']
        efd1 = x1[:, bus_idx, 8]
        gate1 = x1[:, bus_idx, 11]
        print(f"    ω: mean={np.mean(omega1):.6f}, std={np.std(omega1):.6f}")
        print(f"    Efd: mean={np.mean(efd1):.4f}, std={np.std(efd1):.4f}")
        print(f"    Gate: mean={np.mean(gate1):.4f}, std={np.std(gate1):.4f}")
        
        print(f"\n  Bus {bus_idx} - Experiment 2 (Iq injection):")
        print(f"    Vd: mean={np.mean(Vd2):.4f}, std={np.std(Vd2):.4f}, range=[{np.min(Vd2):.4f}, {np.max(Vd2):.4f}]")
        print(f"    Vq: mean={np.mean(Vq2):.4f}, std={np.std(Vq2):.4f}, range=[{np.min(Vq2):.4f}, {np.max(Vq2):.4f}]")
        print(f"    Id: mean={np.mean(Id2):.4f}, std={np.std(Id2):.4f}, range=[{np.min(Id2):.4f}, {np.max(Id2):.4f}]")
        print(f"    Iq: mean={np.mean(Iq2):.4f}, std={np.std(Iq2):.4f}, range=[{np.min(Iq2):.4f}, {np.max(Iq2):.4f}]")
        
        # CRITICAL FIX: Transpose before reshape
        x2 = sol2.y.T.reshape(len(sol2.t), self.n_gen, 13)
        omega2 = x2[:, bus_idx, 1] / self.builder.gen_metadata[bus_idx]['M']
        efd2 = x2[:, bus_idx, 8]
        gate2 = x2[:, bus_idx, 11]
        print(f"    ω: mean={np.mean(omega2):.6f}, std={np.std(omega2):.6f}")
        print(f"    Efd: mean={np.mean(efd2):.4f}, std={np.std(efd2):.4f}")
        print(f"    Gate: mean={np.mean(gate2):.4f}, std={np.std(gate2):.4f}")
        
        print("\n  Generating diagnostic plots...")
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(5, 2, figsize=(14, 12))
        t_plot = sol1.t
        
        # Extract states for target bus - CRITICAL: Transpose first!
        x1 = sol1.y.T.reshape(len(t_plot), self.n_gen, 13)
        x2 = sol2.y.T.reshape(len(t_plot), self.n_gen, 13)
        
        # Exp 1 (Id injection)
        axes[0,0].plot(t_plot, np.degrees(x1[:, bus_idx, 0]))
        axes[0,0].set_ylabel('δ [deg]')
        axes[0,0].set_title('Exp 1: Id Injection')
        axes[0,0].grid(True)
        
        axes[1,0].plot(t_plot, x1[:, bus_idx, 1] / self.builder.gen_metadata[bus_idx]['M'])
        axes[1,0].axhline(1.0, color='r', linestyle='--', alpha=0.3)
        axes[1,0].set_ylabel('ω [pu]')
        axes[1,0].grid(True)
        
        axes[2,0].plot(t_plot, Vd1, label='Vd')
        axes[2,0].plot(t_plot, Vq1, label='Vq')
        axes[2,0].set_ylabel('V [pu]')
        axes[2,0].legend()
        axes[2,0].grid(True)
        
        axes[3,0].plot(t_plot, Id1, label='Id')
        axes[3,0].plot(t_plot, Iq1, label='Iq')
        axes[3,0].set_ylabel('I [pu]')
        axes[3,0].legend()
        axes[3,0].grid(True)
        
        axes[4,0].plot(t_plot, x1[:, bus_idx, 8], label='Efd')
        axes[4,0].plot(t_plot, x1[:, bus_idx, 11], label='Gate')
        axes[4,0].set_ylabel('Controls [pu]')
        axes[4,0].set_xlabel('Time [s]')
        axes[4,0].legend()
        axes[4,0].grid(True)
        
        # Exp 2 (Iq injection)
        axes[0,1].plot(t_plot, np.degrees(x2[:, bus_idx, 0]))
        axes[0,1].set_title('Exp 2: Iq Injection')
        axes[0,1].grid(True)
        
        axes[1,1].plot(t_plot, x2[:, bus_idx, 1] / self.builder.gen_metadata[bus_idx]['M'])
        axes[1,1].axhline(1.0, color='r', linestyle='--', alpha=0.3)
        axes[1,1].grid(True)
        
        axes[2,1].plot(t_plot, Vd2, label='Vd')
        axes[2,1].plot(t_plot, Vq2, label='Vq')
        axes[2,1].legend()
        axes[2,1].grid(True)
        
        axes[3,1].plot(t_plot, Id2, label='Id')
        axes[3,1].plot(t_plot, Iq2, label='Iq')
        axes[3,1].legend()
        axes[3,1].grid(True)
        
        axes[4,1].plot(t_plot, x2[:, bus_idx, 8], label='Efd')
        axes[4,1].plot(t_plot, x2[:, bus_idx, 11], label='Gate')
        axes[4,1].set_xlabel('Time [s]')
        axes[4,1].legend()
        axes[4,1].grid(True)
        
        plt.tight_layout()
        plt.savefig('outputs/imtb_diagnostics.png', dpi=150)
        print("  Diagnostics saved to outputs/imtb_diagnostics.png")
        
        # Remove DC bias before DFT
        Vd1 = self._remove_dc_bias(Vd1)
        Vq1 = self._remove_dc_bias(Vq1)
        Id1 = self._remove_dc_bias(Id1)
        Iq1 = self._remove_dc_bias(Iq1)
        
        Vd2 = self._remove_dc_bias(Vd2)
        Vq2 = self._remove_dc_bias(Vq2)
        Id2 = self._remove_dc_bias(Id2)
        Iq2 = self._remove_dc_bias(Iq2)
        
        for i, f in enumerate(freqs_hz):
            vd1_f = IM.DFT_1f(Vd1, self.fs, f)
            vq1_f = IM.DFT_1f(Vq1, self.fs, f)
            id1_f = IM.DFT_1f(Id1, self.fs, f)
            iq1_f = IM.DFT_1f(Iq1, self.fs, f)
            
            vd2_f = IM.DFT_1f(Vd2, self.fs, f)
            vq2_f = IM.DFT_1f(Vq2, self.fs, f)
            id2_f = IM.DFT_1f(Id2, self.fs, f)
            iq2_f = IM.DFT_1f(Iq2, self.fs, f)
            
            Z_mat = IM.calc_MIMO_Z(vd1_f, vq1_f, id1_f, iq1_f, vd2_f, vq2_f, id2_f, iq2_f)
            Z_mimo[i, :, :] = Z_mat
            
        return freqs_hz, Z_mimo