"""
Diagnostic script to examine initial state derivatives for Thevenin system
"""
import sys
import numpy as np
sys.path.insert(0, '.')
from utils.fault_sim_modular import ModularFaultSimulator

# Initialize simulator
print("Initializing simulator...")
sim = ModularFaultSimulator(
    system_json='test_cases/Thevenin_model/ieee_based_system.json',
    simulation_json='test_cases/Thevenin_model/simulation_nofault.json'
)

# Get initial state
print("Building initial state with power flow...")
x0 = sim.initialize_equilibrium()

print(f"\nInitial state vector length: {len(x0)}")
print(f"Generator states per machine: {sim.states_per_machine}")
print(f"Grid states per grid: {sim.n_grid_states}")

# Compute derivatives at t=0
print("\n" + "="*80)
print("COMPUTING INITIAL DERIVATIVES AT t=0")
print("="*80)

# Extract generator and grid states
n_gen = len(sim.builder.generators)
n_grid = len(sim.builder.grids)

if n_gen > 0:
    x0_gen = x0[:n_gen * sim.states_per_machine].reshape(n_gen, sim.states_per_machine)
if n_grid > 0:
    x0_grid = x0[n_gen * sim.states_per_machine:].reshape(n_grid, sim.n_grid_states)

# Get generator states and grid voltages
gen_states = x0_gen[:, :sim.n_gen_states]  # First 7 states are GENROU states
grid_voltages = None
if n_grid > 0:
    grid_voltages = np.array([x0_grid[i, 0] * np.exp(1j * x0_grid[i, 1])
                              for i in range(n_grid)])

# Solve network
Id, Iq, Vd, Vq = sim.coordinator.solve_network(gen_states, grid_voltages=grid_voltages)

print(f"\nNetwork solution:")
print(f"  Vd={Vd[0]:.6f}, Vq={Vq[0]:.6f}, Vt={np.sqrt(Vd[0]**2 + Vq[0]**2):.6f}")
print(f"  Id={Id[0]:.6f}, Iq={Iq[0]:.6f}")

# Compute derivatives for each component
x_dot = np.zeros_like(x0)

for i in range(n_gen):
    # Generator dynamics (states 0-6)
    gen_core = sim.builder.generators[i]
    gen_meta = sim.builder.gen_metadata[i]
    gen_state = x0_gen[i, :sim.n_gen_states]
    
    # Extract omega correctly: omega = p / M
    p = gen_state[1]
    M = gen_meta['M']
    omega = p / M
    
    u_gen = np.array([Vd[i], Vq[i], Id[i], Iq[i]])
    
    # Get Efd from exciter output
    exc_core = sim.builder.exciters[i]
    exc_meta = sim.builder.exc_metadata[i]
    exc_state = x0_gen[i, sim.n_gen_states:sim.n_gen_states+sim.n_exc_states]
    exc_ports = {
        'Vt': np.sqrt(Vd[i]**2 + Vq[i]**2),
        'Id': Id[i],
        'Iq': Iq[i],
        'XadIfd': gen_state[4]  # psi_f
    }
    Efd = exc_core.output_fn(exc_state, exc_ports, exc_meta)
    
    # Get Pm from governor output
    gov_core = sim.builder.governors[i]
    gov_meta = sim.builder.gov_metadata[i]
    gov_state = x0_gen[i, sim.n_gen_states+sim.n_exc_states:sim.n_gen_states+sim.n_exc_states+sim.n_gov_states]
    gov_ports = {'omega': omega}
    Pm = gov_core.output_fn(gov_state, gov_ports, gov_meta)
    
    # Build ports dict for generator
    gen_ports = {
        'Vd': Vd[i],
        'Vq': Vq[i],
        'Id': Id[i],
        'Iq': Iq[i],
        'Efd': Efd,
        'Tm': Pm
    }
    
    gen_dx = gen_core._dynamics_fn(gen_state, gen_ports, gen_meta)
    x_dot[i*sim.states_per_machine : i*sim.states_per_machine+sim.n_gen_states] = gen_dx
    
    # Exciter dynamics
    exc_ports_dyn = {
        'Vt': np.sqrt(Vd[i]**2 + Vq[i]**2),
        'Id': Id[i],
        'Iq': Iq[i],
        'Vd': Vd[i],
        'Vq': Vq[i],
        'XadIfd': gen_state[4]  # psi_f
    }
    exc_dx = exc_core._dynamics_fn(exc_state, exc_ports_dyn, exc_meta)
    x_dot[i*sim.states_per_machine+sim.n_gen_states : i*sim.states_per_machine+sim.n_gen_states+sim.n_exc_states] = exc_dx
    
    # Governor dynamics
    gov_ports_dyn = {'omega': omega}
    gov_dx = gov_core._dynamics_fn(gov_state, gov_ports_dyn, gov_meta)
    x_dot[i*sim.states_per_machine+sim.n_gen_states+sim.n_exc_states : i*sim.states_per_machine+sim.n_gen_states+sim.n_exc_states+sim.n_gov_states] = gov_dx

# Grid dynamics (algebraic, should be zero)
if n_grid > 0:
    grid_offset = n_gen * sim.states_per_machine
    for i in range(n_grid):
        x_dot[grid_offset + i*sim.n_grid_states : grid_offset + i*sim.n_grid_states + sim.n_grid_states] = 0.0

# Print derivative diagnostics
print(f"\n" + "="*80)
print("DERIVATIVE DIAGNOSTICS")
print("="*80)

state_names = [
    'delta', 'omega', 'psi_d', 'psi_q', 'psi_f', 'psi_kd', 'psi_kq',
    'LG_y', 'LL_exc_x', 'VR', 'VM', 'VB_state',
    'gov_x1', 'gov_x2'
]

max_abs_dx = 0.0
print(f"\n{'State':<12} {'Value':>12} {'dx/dt':>15}")
print("-" * 42)

for i, (name, val, dx) in enumerate(zip(state_names, x0[:14], x_dot[:14])):
    if abs(dx) > 1e-3:
        print(f"{name:<12} {val:12.6f} {dx:15.6e} <--")
    else:
        print(f"{name:<12} {val:12.6f} {dx:15.6e}")
    max_abs_dx = max(max_abs_dx, abs(dx))

print("-" * 42)
print(f"Max |dx/dt|: {max_abs_dx:.6e}")

# Check specific values
print(f"\n" + "="*80)
print("EQUILIBRIUM CHECK")
print("="*80)

omega = x0_gen[0, 1]
psi_d = x0_gen[0, 2]
psi_q = x0_gen[0, 3]
psi_f = x0_gen[0, 4]

# Compute expected stator fluxes
omega_b = gen_meta['omega_b']
ra = gen_meta['ra']
psi_d_expected = (Vq[0] + ra * Iq[0]) / omega_b
psi_q_expected = -(Vd[0] + ra * Id[0]) / omega_b

print(f"Generator state:")
print(f"  p (momentum) = {x0_gen[0, 1]:.6f}")
print(f"  omega = p/M = {x0_gen[0, 1]/gen_meta['M']:.6f} (should be 1.0)")
print(f"  psi_d = {psi_d:.6f} (expected {psi_d_expected:.6f})")
print(f"  psi_q = {psi_q:.6f} (expected {psi_q_expected:.6f})")

# Check Pm vs Pe
Pe = Vd[0] * Id[0] + Vq[0] * Iq[0]
print(f"\nPower balance:")
print(f"  Pm = {Pm:.6f}")
print(f"  Pe = {Pe:.6f}")
print(f"  Pm - Pe = {Pm - Pe:.6f}")

# Check governor states
gov_x1 = x0_gen[0, sim.n_gen_states + sim.n_exc_states]
gov_x2 = x0_gen[0, sim.n_gen_states + sim.n_exc_states + 1]
VMIN = sim.builder.gov_metadata[0]['VMIN']
VMAX = sim.builder.gov_metadata[0]['VMAX']
Pref = sim.builder.gov_metadata[0]['Pref']
R = sim.builder.gov_metadata[0]['R']

print(f"\nGovernor state:")
print(f"  x1 = {gov_x1:.6f}")
print(f"  x2 = {gov_x2:.6f} (should equal Pm={Pm:.6f})")
print(f"  Pref = {Pref:.6f}")
print(f"  VMIN = {VMIN}, VMAX = {VMAX}")
print(f"  Gate output = clip(Pref + (omega-1)/R, VMIN, VMAX)")
speed_error = omega - 1.0
gate_cmd = Pref + speed_error / R
gate_out = np.clip(gate_cmd, VMIN, VMAX)
print(f"             = clip({Pref:.6f} + {speed_error:.6f}/{R}, {VMIN}, {VMAX})")
print(f"             = {gate_out:.6f}")

# Check ESST3A
LG_y = x0_gen[0, sim.n_gen_states]
LL_exc_x = x0_gen[0, sim.n_gen_states + 1]
VR = x0_gen[0, sim.n_gen_states + 2]
VM = x0_gen[0, sim.n_gen_states + 3]
VB_state = x0_gen[0, sim.n_gen_states + 4]

Vt = np.sqrt(Vd[0]**2 + Vq[0]**2)

print(f"\nESST3A state:")
print(f"  LG_y = {LG_y:.6f} (actual Vt = {Vt:.6f})")
print(f"  VB_state = {VB_state:.6f}")
print(f"  VM = {VM:.6f}")
print(f"  VR = {VR:.6f}")
print(f"  Efd = {Efd:.6f}")

# Check Vref
Vref = sim.builder.exc_metadata[0]['vref']
print(f"\nESST3A parameters:")
print(f"  Vref = {Vref:.6f}")
print(f"  vi (error) = Vref - Vt = {Vref - Vt:.6f}")
KA = sim.builder.exc_metadata[0]['KA']
TC = sim.builder.exc_metadata[0]['TC']
TB = sim.builder.exc_metadata[0]['TB']
VIMIN = sim.builder.exc_metadata[0]['VIMIN']
VIMAX = sim.builder.exc_metadata[0]['VIMAX']
vil = np.clip(Vref - Vt, VIMIN, VIMAX)
LL_exc_y = (TB/TC) * (vil - LL_exc_x) + LL_exc_x
VR_unlimited = KA * LL_exc_y
print(f"  vil = clip(vi, {VIMIN}, {VIMAX}) = {vil:.6f}")
print(f"  LL_exc_y = (TB/TC)*(vil-LL_exc_x) + LL_exc_x = {LL_exc_y:.6f}")
print(f"  VR_unlimited = KA * LL_exc_y = {VR_unlimited:.6f}")
print(f"  VR (state) = {VR:.6f}")
print(f"  VR error = VR_unlimited - VR = {VR_unlimited - VR:.6f}")
