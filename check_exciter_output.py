"""Check exciter output values during simulation"""
import numpy as np
import matplotlib.pyplot as plt
from utils.fault_sim_modular import ModularFaultSimulator

# Initialize and run
sim = ModularFaultSimulator('test_cases/Thevenin_model/ieee_based_system.json')
sim.disable_fault()
x0 = sim.initialize_equilibrium()

# Run short simulation
sol = sim.simulate(x0, t_end=5.0, n_points=500)

# Extract exciter states and calculate Efd
gen_size = sim.n_gen * sim.states_per_machine
x_gen_hist = sol.y[:gen_size, :].T
x_hist = x_gen_hist.reshape(-1, sim.n_gen, sim.states_per_machine)

# Calculate Efd from states
t = sol.t
Efd_hist = np.zeros((len(t), sim.n_gen))

for idx in range(len(t)):
    for g in range(sim.n_gen):
        exc_offset = sim.n_gen_states
        exc_states = x_hist[idx, g, exc_offset:exc_offset+sim.n_exc_states]
        
        if sim.exciter_type == 'ESST3A':
            # Efd = VB_state * VM (with same limit as in dynamics)
            VM = exc_states[3]
            VB_state = exc_states[4]
            Efd_raw = VB_state * VM
            # Apply same hard limit as during simulation (5 pu typical maximum)
            Efd_hist[idx, g] = np.clip(Efd_raw, 0.0, 5.0)
        else:
            Efd_hist[idx, g] = exc_states[1]

print("\n" + "="*70)
print("EXCITER OUTPUT ANALYSIS")
print("="*70)

for g in range(sim.n_gen):
    Efd_max = np.max(Efd_hist[:, g])
    Efd_min = np.min(Efd_hist[:, g])
    Efd_mean = np.mean(Efd_hist[:, g])
    
    print(f"\nGenerator {g}:")
    print(f"  Efd initial: {Efd_hist[0, g]:.3f} pu")
    print(f"  Efd final:   {Efd_hist[-1, g]:.3f} pu")
    print(f"  Efd max:     {Efd_max:.3f} pu")
    print(f"  Efd min:     {Efd_min:.3f} pu")
    print(f"  Efd mean:    {Efd_mean:.3f} pu")
    
    if Efd_max > 10.0:
        print(f"  WARNING: Efd exceeds 10 pu (unrealistic!)")
    elif Efd_max > 5.0:
        print(f"  CAUTION: Efd exceeds 5 pu (high excitation)")
    else:
        print(f"  OK: Efd within realistic limits (<5 pu)")

# Plot
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))

# Efd
for g in range(sim.n_gen):
    ax1.plot(t, Efd_hist[:, g], label=f'Gen {g+1}')
ax1.axhline(y=5.0, color='r', linestyle='--', label='5 pu limit')
ax1.set_ylabel('Efd (pu)')
ax1.set_title('Exciter Field Voltage')
ax1.legend()
ax1.grid(True)

# Omega
for g in range(sim.n_gen):
    omega = x_hist[:, g, 1] / sim.builder.gen_metadata[g]['M']
    ax2.plot(t, omega, label=f'Gen {g+1}')
ax2.axhline(y=1.0, color='k', linestyle='--', alpha=0.3)
ax2.set_ylabel('omega (pu)')
ax2.set_xlabel('Time (s)')
ax2.set_title('Rotor Speed')
ax2.legend()
ax2.grid(True)

plt.tight_layout()
plt.savefig('outputs/exciter_diagnostic.png', dpi=150)
print(f"\nPlot saved: outputs/exciter_diagnostic.png")
print("="*70)
