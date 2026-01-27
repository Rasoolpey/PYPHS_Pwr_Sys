"""Debug script to understand network solution"""
import sys
sys.path.insert(0, '.')
import numpy as np
np.set_printoptions(precision=6, suppress=True)
from utils.system_builder import PowerSystemBuilder
from utils.system_coordinator import PowerSystemCoordinator

# Build system
builder = PowerSystemBuilder('test_cases/Kundur_System/kundur_full.json')
builder.build_all_components()

coordinator = PowerSystemCoordinator(builder)

print("=== Bus Mapping ===")
print(f"Bus idx to internal: {coordinator.bus_idx_to_internal}")
print(f"Gen to bus: {coordinator.gen_to_bus}")
print(f"Gen bus internal: {coordinator.gen_bus_internal}")
print(f"Unique gen buses: {coordinator.unique_gen_buses}")
print(f"Gen bus to reduced: {coordinator.gen_bus_to_reduced}")

print("\n=== Ybus Base Shape ===")
print(f"Ybus_base shape: {coordinator.Ybus_base.shape}")
print(f"Ybus_reduced shape: {coordinator.Ybus_reduced.shape}")

print("\n=== Ybus_reduced (4x4 for generator buses) ===")
print("Magnitude:")
print(np.abs(coordinator.Ybus_reduced))
print("\nAngle (degrees):")
print(np.degrees(np.angle(coordinator.Ybus_reduced)))

# Test with known initial conditions from JSON
print("\n=== Testing Network Solution ===")

# Initial angles from Bus data
delta_init = np.array([0.5702549, 0.3687462, 0.1853173, 0.4623587])  # radians

# Create gen_states
n_gen = 4
gen_states = np.zeros((n_gen, 7))

for i in range(n_gen):
    M = builder.gen_metadata[i]['M']
    gen_states[i, 0] = delta_init[i]  # delta
    gen_states[i, 1] = M * 1.0  # p = M * omega (omega=1.0 at equilibrium)
    gen_states[i, 4] = 1.5  # psi_f - try higher value to get more power

print(f"Input gen_states:")
print(f"  deltas (deg): {np.degrees(gen_states[:, 0])}")
print(f"  psi_f: {gen_states[:, 4]}")

# Solve network
Id, Iq, Vd, Vq = coordinator.solve_network(gen_states)

print(f"\nNetwork solution:")
print(f"  Id: {Id}")
print(f"  Iq: {Iq}")
print(f"  Vd: {Vd}")
print(f"  Vq: {Vq}")

# Calculate power and voltage
P = Vd * Id + Vq * Iq
Q = Vq * Id - Vd * Iq
V_mag = np.sqrt(Vd**2 + Vq**2)

print(f"\nCalculated:")
print(f"  P: {P}")
print(f"  Q: {Q}")
print(f"  V: {V_mag}")

# Check generator internal reactance
print("\n=== Generator Parameters ===")
for i in range(n_gen):
    meta = builder.gen_metadata[i]
    Xd2 = meta.get('xd2', 0.25)
    Sn = meta.get('Sn', 900.0)
    S_system = builder.S_system
    Xd2_sys = Xd2 * (S_system / Sn)
    print(f"Gen {i}: Xd''={Xd2} pu (machine), Xd''_sys={Xd2_sys:.4f} pu (100 MVA base)")

# Check the Ybus_base at generator and HV buses
print("\n=== Ybus_base diagonal ===")
print(f"Generator buses (1-4, internal 0-3):")
for i in range(4):
    print(f"  Bus {i}: Y_ii = {coordinator.Ybus_base[i,i]:.4f}")
print(f"HV buses (5-10, internal 4-9):")
for i in range(4, 10):
    print(f"  Bus {i}: Y_ii = {coordinator.Ybus_base[i,i]:.4f}")

# Check what the Kron reduction is doing
print("\n=== Kron reduction details ===")
gen_buses = coordinator.unique_gen_buses
load_buses = [i for i in range(coordinator.n_bus) if i not in gen_buses]
print(f"Gen buses: {gen_buses}")
print(f"Load buses: {load_buses}")

Ygg = coordinator.Ybus_base[np.ix_(gen_buses, gen_buses)]
Ygl = coordinator.Ybus_base[np.ix_(gen_buses, load_buses)]
Ylg = coordinator.Ybus_base[np.ix_(load_buses, gen_buses)]
Yll = coordinator.Ybus_base[np.ix_(load_buses, load_buses)]

print(f"\nYgg shape: {Ygg.shape}")
print(f"Ygg diagonal magnitudes: {np.abs(np.diag(Ygg))}")
print(f"\nYgl shape: {Ygl.shape}")
print(f"Ygl[0,:] magnitudes (Gen 0 to load buses): {np.abs(Ygl[0,:])}")
print(f"\nYll shape: {Yll.shape}")
print(f"Yll diagonal magnitudes: {np.abs(np.diag(Yll))}")

# After adding loads
print(f"\nLoads being added to Yll diagonal:")
for i, lb in enumerate(load_buses):
    P = coordinator.load_P[lb]
    Q = coordinator.load_Q[lb]
    S = P + 1j * Q
    if abs(S) > 1e-6:
        Y_load = np.conj(S)
        print(f"  Load bus {lb}: P={P:.2f}, Q={Q:.2f}, Y_load={Y_load:.4f}")

# Check the actual metadata values
print("\n=== Metadata values (should be on system base) ===")
for i in range(n_gen):
    meta = builder.gen_metadata[i]
    print(f"Gen {i}: xd2={meta.get('xd2'):.6f}, xd1={meta.get('xd1'):.6f}, xl={meta.get('xl'):.6f}")
    xd2 = meta.get('xd2')
    xd1 = meta.get('xd1')
    xl = meta.get('xl')
    if (xd1 - xl) > 1e-6:
        gd1_calc = (xd2 - xl) / (xd1 - xl)
        print(f"        gd1=(xd2-xl)/(xd1-xl) = ({xd2}-{xl})/({xd1}-{xl}) = {gd1_calc:.4f}")
    print(f"        gd1 from meta={meta.get('gd1'):.4f}")

# Trace through E'' calculation manually
print("\n=== Manual E'' calculation ===")
for i in range(n_gen):
    meta = builder.gen_metadata[i]
    psi_f = gen_states[i, 4]
    xd2 = meta.get('xd2')
    xd1 = meta.get('xd1')
    xl = meta.get('xl')
    delta = gen_states[i, 0]

    Eq2 = (xd2 - xl) / (xd1 - xl) * psi_f
    Ed2 = 0  # psi_kq = 0
    E_internal = Ed2 + 1j * Eq2

    # Park transform
    e_d = np.real(E_internal)
    e_q = np.imag(E_internal)
    E_R = e_d * np.sin(delta) + e_q * np.cos(delta)
    E_I = -e_d * np.cos(delta) + e_q * np.sin(delta)
    E_sys = E_R + 1j * E_I

    y_gen = 1.0 / (1j * xd2)

    print(f"Gen {i}: psi_f={psi_f}, Eq2={Eq2:.4f}")
    print(f"        E_internal={E_internal}")
    print(f"        delta={np.degrees(delta):.2f} deg")
    print(f"        E_sys={E_sys} (mag={np.abs(E_sys):.4f}, ang={np.degrees(np.angle(E_sys)):.2f} deg)")
    print(f"        y_gen={y_gen} (mag={np.abs(y_gen):.4f})")

# Let's check what E'' should be to get P=7 pu
print("\n=== Required E'' for P=7 pu (Classical Model) ===")
# P = E'' * V * sin(delta) / X''
# For P=7 pu, V=1, delta=0.5, X''=0.0278:
# E'' = P * X'' / (V * sin(delta))
for i in range(n_gen):
    P_target = [7.459, 7.0, 7.0, 7.0][i]
    delta = delta_init[i]
    Xd2_sys = builder.gen_metadata[i].get('xd2')  # Already on system base
    if np.sin(delta) > 0.01:
        E_required = P_target * Xd2_sys / (1.0 * np.sin(delta))
        print(f"Gen {i}: P_target={P_target}, delta={np.degrees(delta):.1f} deg, Xd''_sys={Xd2_sys:.4f}")
        print(f"         Required E''={E_required:.4f} pu")
