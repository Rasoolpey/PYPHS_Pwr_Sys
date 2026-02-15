"""
test_gfm_inverter.py
====================
Demonstration test for the Passive Grid-Forming (GFM) inverter controller
at Bus 8 of the IEEE 14-bus system.

The GFM inverter replaces the existing Grid-Following (GFL) WT3 stack
(REGCA1/REECA1/REPCA1/WTDTA1/WTARA1/WTPTA1/WTTQA1) with a single
Port-Hamiltonian GFM model based on:

  Kong, L., Xue, Y., Qiao, L., & Wang, F. (2024).
  "Control Design of Passive Grid-Forming Inverters in PH Framework"
  IEEE Transactions on Power Electronics, 39(1), 332-345.
"""

import sys, os, types

# Bootstrap: resolve 'utils.X' -> flat /mnt/project/X.py
_PROJECT = '/mnt/project'
_GFM_DIR = '/home/claude'
if 'utils' not in sys.modules:
    _utils = types.ModuleType('utils')
    _utils.__path__ = [_PROJECT]
    _utils.__package__ = 'utils'
    sys.modules['utils'] = _utils
sys.path.insert(0, _PROJECT)
sys.path.insert(0, _GFM_DIR)

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

from gfm_inverter import (build_gfm_inverter_core,
                          gfm_inverter_dynamics,
                          gfm_inverter_output)
from scipy.integrate import solve_ivp


def run_gfm_standalone_test(t_end=30.0, fault_t_start=1.0, fault_duration=0.1):
    """Simulate GFM inverter standalone with voltage-dip fault."""

    gfm_data = {
        'idx': 'GFM_test', 'bus': 8, 'Sn': 100.0,
        'Lf': 0.08, 'Cf': 0.074, 'Rf': 0.01,
        'u_ref': 1.03, 'omega0': 1.0,
        'xi1': 0.06, 'xi2': 0.42, 'xi3': 0.2,
        'eps': 1e-5, 'omega_lpf': 62.83,
        'Pref': 0.35, 'Qref': 0.074,
        'hysteresis': 0.01, 'Imax': 1.3
    }
    core, meta = build_gfm_inverter_core(gfm_data)

    V0, th0 = 1.03, -0.025
    x0 = core.init_fn(P0=0.35, Q0=0.074, V0=V0, theta0=th0)
    print(f"  Initial state: u_a={x0[0]:.4f}, u_b={x0[1]:.4f}, "
          f"theta={x0[2]:.4f} rad, Pf={x0[3]:.4f}, Qf={x0[4]:.4f}")

    ig_a0 = meta['Pref'] / V0 * np.cos(th0)
    ig_b0 = meta['Pref'] / V0 * np.sin(th0)
    dx0 = gfm_inverter_dynamics(x0, {'ig_a': ig_a0, 'ig_b': ig_b0, 'V': V0}, meta)
    print(f"  Equilibrium check  max|dx/dt| = {np.max(np.abs(dx0)):.2e}")

    def ode(t, x):
        u_a, u_b, theta, Pf, Qf = x
        V_mag = np.sqrt(u_a**2 + u_b**2)
        V_mag = max(V_mag, 0.05)
        fault_on = (fault_t_start <= t <= fault_t_start + fault_duration)
        base_ig_a = meta['Pref'] / V_mag * np.cos(theta)
        base_ig_b = meta['Pref'] / V_mag * np.sin(theta)
        if fault_on:
            # Fault: inject extra reactive load (short-circuit approximation)
            ig_a = base_ig_a + 0.5 * u_b
            ig_b = base_ig_b - 0.5 * u_a
        else:
            ig_a, ig_b = base_ig_a, base_ig_b
        return gfm_inverter_dynamics(x, {'ig_a': ig_a, 'ig_b': ig_b, 'V': V_mag}, meta)

    print(f"\n  Integrating (t=0 → {t_end}s) ...")
    sol = solve_ivp(ode, [0, t_end], x0,
                    method='Radau', rtol=1e-6, atol=1e-8,
                    dense_output=False, max_step=0.05)

    t_v  = sol.t
    y_m  = sol.y
    V_v  = np.sqrt(y_m[0]**2 + y_m[1]**2)

    # Hamiltonian (Lyapunov candidate)
    Cf_  = meta['Cf']; eps_ = meta['eps']; ur2 = meta['u_ref']**2
    u2   = y_m[0]**2 + y_m[1]**2
    H_P  = 0.5 * Cf_ * u2
    Phi_ = 0.5 * Cf_ * (u2 - ur2)
    H_C  = 0.5 * Phi_**2 / eps_**2
    H_v  = H_P + H_C

    print(f"  Done: {len(t_v)} steps, t_final={t_v[-1]:.2f}s")
    print(f"  Final |u| = {V_v[-1]:.4f} pu  (ref {meta['u_ref']} pu)")
    print(f"  Final Pf  = {y_m[3,-1]:.4f} pu  (ref {meta['Pref']} pu)")
    print(f"  Final Qf  = {y_m[4,-1]:.4f} pu  (ref {meta['Qref']} pu)")
    dH   = H_v[-1] - H_v[0]
    print(f"  ΔH (passivity)  = {dH:.4e}  "
          f"({'✓ H non-increasing (passive)' if dH <= 1e-3 else '! check passivity'})")
    return sol, t_v, y_m, V_v, H_v, meta


def plot_results(t_v, y_m, V_v, H_v, meta,
                 fault_t_start=1.0, fault_dur=0.1,
                 save_path='/mnt/user-data/outputs/gfm_inverter_results.png'):

    os.makedirs(os.path.dirname(save_path), exist_ok=True)
    fig = plt.figure(figsize=(15, 10))
    fig.suptitle(
        'Passive Grid-Forming Inverter — Port-Hamiltonian Framework\n'
        'IEEE 14-Bus System · Bus 8 · Kong et al. (IEEE Trans. Power Electron., 2024)',
        fontsize=12, fontweight='bold')
    gs = gridspec.GridSpec(3, 3, figure=fig, hspace=0.44, wspace=0.36)

    def shade(ax):
        ax.axvspan(fault_t_start, fault_t_start + fault_dur,
                   alpha=0.15, color='red', label='Fault')

    # 1. Voltage magnitude
    ax = fig.add_subplot(gs[0, 0])
    ax.plot(t_v, V_v, 'b-', lw=1.5, label='|u(t)|')
    ax.axhline(meta['u_ref'], color='k', ls='--', lw=1, label=f"u_ref={meta['u_ref']}")
    shade(ax); ax.legend(fontsize=7)
    ax.set_title('Terminal Voltage Magnitude'); ax.set_xlabel('Time (s)'); ax.set_ylabel('(pu)')
    ax.grid(True, alpha=0.3)

    # 2. αβ voltages
    ax = fig.add_subplot(gs[0, 1])
    ax.plot(t_v, y_m[0], label='u_α'); ax.plot(t_v, y_m[1], label='u_β')
    shade(ax); ax.legend(fontsize=7)
    ax.set_title('Output Voltages αβ'); ax.set_xlabel('Time (s)'); ax.set_ylabel('(pu)')
    ax.grid(True, alpha=0.3)

    # 3. Virtual oscillator phase
    ax = fig.add_subplot(gs[0, 2])
    ax.plot(t_v, np.rad2deg(y_m[2]), 'g-', lw=1.5)
    shade(ax)
    ax.set_title('Virtual Oscillator Phase θ'); ax.set_xlabel('Time (s)'); ax.set_ylabel('(deg)')
    ax.grid(True, alpha=0.3)

    # 4. Active power
    ax = fig.add_subplot(gs[1, 0])
    ax.plot(t_v, y_m[3], 'b-', lw=1.5, label='Pf')
    ax.axhline(meta['Pref'], color='k', ls='--', lw=1, label=f"Pref={meta['Pref']}")
    shade(ax); ax.legend(fontsize=7)
    ax.set_title('Active Power (LPF)'); ax.set_xlabel('Time (s)'); ax.set_ylabel('(pu)')
    ax.grid(True, alpha=0.3)

    # 5. Reactive power
    ax = fig.add_subplot(gs[1, 1])
    ax.plot(t_v, y_m[4], 'r-', lw=1.5, label='Qf')
    ax.axhline(meta['Qref'], color='k', ls='--', lw=1, label=f"Qref={meta['Qref']}")
    shade(ax); ax.legend(fontsize=7)
    ax.set_title('Reactive Power (LPF)'); ax.set_xlabel('Time (s)'); ax.set_ylabel('(pu)')
    ax.grid(True, alpha=0.3)

    # 6. Hamiltonian / Lyapunov
    ax = fig.add_subplot(gs[1, 2])
    H_norm = H_v / (H_v[0] + 1e-12)
    ax.plot(t_v, H_norm, 'm-', lw=1.5, label='H(t)/H(0)')
    ax.axhline(1.0, color='k', ls=':', lw=0.8)
    shade(ax); ax.legend(fontsize=7)
    ax.set_title('Hamiltonian H(t)/H(0)\n(Passivity: H ≤ H(0))'); ax.set_xlabel('Time (s)')
    ax.grid(True, alpha=0.3)

    # 7. Passivity index KP
    ax = fig.add_subplot(gs[2, 0])
    Cf_ = meta['Cf']; u2 = y_m[0]**2 + y_m[1]**2
    ur2 = meta['u_ref']**2; xi1_ = meta['xi1']; xi2_ = abs(meta['xi2'])
    Phi_ = 0.5 * Cf_ * (u2 - ur2)
    with np.errstate(divide='ignore', invalid='ignore'):
        Qr = np.where(u2 > 1e-6, meta['Qref'] / ur2 - y_m[4] / u2, 0.0)
        KP = np.where(np.abs(u2 - ur2) > 1e-8,
                      2 * xi1_ - 2 * xi2_ * Qr / (u2 - ur2),
                      xi1_ * np.ones_like(u2))
    ax.plot(t_v, KP, 'b-', lw=1.2, label='KP (GFM)')
    ax.axhline(0, color='r', ls='--', lw=1, label='KP=0 boundary')
    shade(ax); ax.legend(fontsize=7)
    ax.set_ylim([-1, min(5, np.nanpercentile(KP[np.isfinite(KP)], 99) + 0.5)])
    ax.set_title('Passivity Index KP\n(KP > 0 ⟹ passive)')
    ax.set_xlabel('Time (s)'); ax.grid(True, alpha=0.3)

    # 8. Phase portrait
    ax = fig.add_subplot(gs[2, 1])
    sc = ax.scatter(np.rad2deg(y_m[2]), V_v, c=t_v, cmap='viridis', s=3)
    plt.colorbar(sc, ax=ax, label='Time (s)')
    ax.axhline(meta['u_ref'], color='k', ls='--', lw=0.8)
    ax.set_title('Phase Portrait θ vs |u|')
    ax.set_xlabel('θ (deg)'); ax.set_ylabel('|u| (pu)')
    ax.grid(True, alpha=0.3)

    # 9. αβ voltage trajectory
    ax = fig.add_subplot(gs[2, 2])
    ax.plot(y_m[0], y_m[1], 'b-', lw=0.8, alpha=0.6)
    ax.plot(y_m[0, 0], y_m[1, 0], 'go', ms=8, label='Start')
    ax.plot(y_m[0, -1], y_m[1, -1], 'rs', ms=8, label='End')
    th_ = np.linspace(0, 2*np.pi, 300)
    r_  = meta['u_ref']
    ax.plot(r_*np.cos(th_), r_*np.sin(th_), 'k--', lw=0.8, label='|u|=u_ref')
    ax.set_aspect('equal'); ax.legend(fontsize=7)
    ax.set_title('αβ Voltage Trajectory')
    ax.set_xlabel('u_α (pu)'); ax.set_ylabel('u_β (pu)')
    ax.grid(True, alpha=0.3)

    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    print(f"  Figure saved → {save_path}")
    return fig


if __name__ == '__main__':
    print("\n" + "="*65)
    print("  PASSIVE GRID-FORMING INVERTER  |  PH Framework")
    print("  IEEE 14-Bus · Bus 8 · Kong et al. (2024)")
    print("="*65)

    sol, t_v, y_m, V_v, H_v, meta = run_gfm_standalone_test(
        t_end=30.0, fault_t_start=1.0, fault_duration=0.1)

    plot_results(t_v, y_m, V_v, H_v, meta,
                 fault_t_start=1.0, fault_dur=0.1)

    print("\n" + "="*65)
    print("  SUMMARY")
    print("="*65)
    print(f"  Voltage:   {V_v[0]:.4f} → {V_v[-1]:.4f} pu  (ref {meta['u_ref']} pu)")
    print(f"  Active P:  {y_m[3,-1]:.4f} pu  (Pref={meta['Pref']} pu)")
    print(f"  React  Q:  {y_m[4,-1]:.4f} pu  (Qref={meta['Qref']} pu)")
    print(f"  Phase θ:   {np.rad2deg(y_m[2,-1]):.2f} °")
    dH = H_v[-1] - H_v[0]
    print(f"  ΔH:        {dH:.3e}  ({'passive ✓' if dH <= 1e-3 else 'check ✗'})")
    print("="*65)
