"""
Lightweight animation generator for trajectory combination demo.
Generates small, fast GIFs with ~60 frames each.
"""
import os, sys, math
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from robot_params import RobotParams
from pid_controller import PIDController, PIDGains
from imu_simulator import IMUSimulator
from body_posture import inverse_kinematics
from disturbance import Disturbance

plt.rcParams.update({
    'figure.facecolor': '#1a1a2e', 'axes.facecolor': '#16213e',
    'axes.edgecolor': '#e94560', 'axes.labelcolor': '#eaeaea',
    'text.color': '#eaeaea', 'xtick.color': '#aaa', 'ytick.color': '#aaa',
    'grid.color': '#2a2a4a', 'grid.alpha': 0.5,
    'legend.facecolor': '#1a1a2e', 'legend.edgecolor': '#e94560',
    'font.size': 10,
})

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")
os.makedirs(OUT, exist_ok=True)

# ── Generate trajectory ──────────────────────────────────────────
def make_traj(n_stance=300, n_swing=150):
    stride, z_st, lift = 0.045, -0.170, 0.040
    legs = {
        'left-front':  (0.125,  0.135, 0.0),
        'left-behind': (-0.125, 0.135, 0.5),
        'right-front': (0.125, -0.135, 0.5),
        'right-behind':(-0.125,-0.135, 0.0),
    }
    T = n_swing + n_stance
    trajs = {}
    for name, (xc, y, phase) in legs.items():
        xf, xb = xc + stride/2, xc - stride/2
        swing = np.zeros((n_swing, 3))
        swing[:,0] = np.linspace(xb, xf, n_swing)
        swing[:,1] = y
        swing[:,2] = z_st + lift * np.sin(np.linspace(0, np.pi, n_swing))
        stance = np.zeros((n_stance, 3))
        stance[:,0] = np.linspace(xf, xb, n_stance)
        stance[:,1] = y
        stance[:,2] = z_st
        full = np.vstack([swing, stance])
        trajs[name] = np.roll(full, round(T*phase), axis=0)
    return trajs, T

def rot_inv(p, dr, dp):
    """Paper formula: p_t = R_inv · (p_c - p0) + p0, with p0=[0,0,0]"""
    r, p_ = -dr, -dp
    cr, sr, cp, sp = math.cos(r), math.sin(r), math.cos(p_), math.sin(p_)
    R = np.array([[cp, sr*sp, cr*sp],[0, cr, -sr],[-sp, sr*cp, cr*cp]])
    return R @ p

# ── Run simulation ───────────────────────────────────────────────
def run_sim(T_sec=3.0, dt=0.005):
    """Coarser timestep for faster sim."""
    params = RobotParams(dt=dt)
    trajs, T_cyc = make_traj()
    gains = PIDGains(Kp=5.0, Ki=1.0, Kd=0.3)
    pid_r = PIDController(gains, dt, 0.087, 0.03, "R")
    pid_p = PIDController(gains, dt, 0.087, 0.03, "P")
    imu = IMUSimulator(noise_std_roll=0.001, noise_std_pitch=0.001, dt=dt)
    dist = Disturbance(dist_type="sinusoidal", roll_amplitude=np.radians(3),
                       pitch_amplitude=np.radians(2), roll_frequency=0.5,
                       pitch_frequency=0.3)

    n = int(T_sec / dt)
    legs = ['left-front','left-behind','right-front','right-behind']
    tau = 0.05; alpha = dt/(tau+dt)
    body_r, body_p = 0.0, 0.0

    H = {'t':[], 'br':[], 'bp':[], 'pr':[], 'pp':[], 'dr':[], 'dp':[]}
    for lg in legs:
        H[f'{lg}_ox']=[]; H[f'{lg}_oz']=[]
        H[f'{lg}_ax']=[]; H[f'{lg}_az']=[]
        H[f'{lg}_ot2']=[]; H[f'{lg}_ft2']=[]

    gf = 0
    for i in range(n):
        t = i*dt
        dr, dp = dist.get(t)
        mr, mp = imu.measure(body_r, body_p, t)
        cr = pid_r.compute(0.0, mr, t)
        cp_ = pid_p.compute(0.0, mp, t)

        for lg in legs:
            po = trajs[lg][gf].copy()
            pa = rot_inv(po, cr, cp_)
            ao = inverse_kinematics(po[0],po[1],po[2], lg, params)
            af = inverse_kinematics(pa[0],pa[1],pa[2], lg, params)
            H[f'{lg}_ox'].append(po[0]); H[f'{lg}_oz'].append(po[2])
            H[f'{lg}_ax'].append(pa[0]); H[f'{lg}_az'].append(pa[2])
            H[f'{lg}_ot2'].append(ao[1]); H[f'{lg}_ft2'].append(af[1])

        body_r += alpha*(dr + cr - body_r)
        body_p += alpha*(dp + cp_ - body_p)

        H['t'].append(t); H['br'].append(body_r); H['bp'].append(body_p)
        H['pr'].append(cr); H['pp'].append(cp_)
        H['dr'].append(dr); H['dp'].append(dp)
        gf = (gf+1) % (150+300)

    return {k: np.array(v) for k,v in H.items()}


# ── Animation 1: D-shape foot trajectory ─────────────────────────
def anim_dshape(H):
    print("  Generating D-shape foot trajectory animation...")
    ds = 10  # downsample: 600/10 = 60 frames
    fig, axes = plt.subplots(2, 2, figsize=(10, 8))
    fig.suptitle("D-Shape: Offline (orange) vs PID-Adjusted (blue)",
                 fontsize=12, fontweight='bold', y=0.98)

    legs = ['left-front','left-behind','right-front','right-behind']
    titles = ['Left-Front','Left-Behind','Right-Front','Right-Behind']
    axs = [axes[0,0], axes[0,1], axes[1,0], axes[1,1]]

    data = {}
    for lg in legs:
        data[lg] = {
            'ox': H[f'{lg}_ox'][::ds]*1000, 'oz': H[f'{lg}_oz'][::ds]*1000,
            'ax': H[f'{lg}_ax'][::ds]*1000, 'az': H[f'{lg}_az'][::ds]*1000,
        }

    artists = []
    for ax, title in zip(axs, titles):
        ax.set_xlim(80,170); ax.set_ylim(-185,-115)
        ax.set_xlabel('X (mm)'); ax.set_ylabel('Z (mm)')
        ax.set_title(title, fontsize=10, fontweight='bold')
        ax.grid(True)
        ot, = ax.plot([],[], '-', color='#ffa502', lw=1.5, alpha=0.6)
        at, = ax.plot([],[], '-', color='#54a0ff', lw=1.5, alpha=0.6)
        od, = ax.plot([],[], 'o', color='#ffa502', ms=7, label='Offline')
        ad, = ax.plot([],[], 's', color='#54a0ff', ms=7, label='Adjusted')
        ax.legend(fontsize=7, loc='upper right')
        artists.append((ot,at,od,ad))

    n_frames = len(H['t'][::ds])
    trail = 60

    def animate(i):
        s0 = max(0,i-trail)
        for j, lg in enumerate(legs):
            d = data[lg]
            s = slice(s0, i+1)
            artists[j][0].set_data(d['ox'][s], d['oz'][s])
            artists[j][1].set_data(d['ax'][s], d['az'][s])
            artists[j][2].set_data([d['ox'][i]], [d['oz'][i]])
            artists[j][3].set_data([d['ax'][i]], [d['az'][i]])
        return ()

    ani = animation.FuncAnimation(fig, animate, frames=n_frames, interval=80, blit=True)
    fig.tight_layout(rect=[0,0,1,0.95])
    path = os.path.join(OUT, 'foot_trajectory_v2.gif')
    ani.save(path, writer='pillow', fps=12)
    plt.close(fig)
    print(f"  ✓ foot_trajectory_v2.gif ({os.path.getsize(path)//1024} KB, {n_frames} frames)")


# ── Animation 2: Full pipeline ───────────────────────────────────
def anim_pipeline(H):
    print("  Generating pipeline animation...")
    ds = 10
    t = H['t'][::ds]
    n = len(t)
    lg = 'left-front'

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle("Pipeline: Offline D-Shape + PID → Final Trajectory",
                 fontsize=12, fontweight='bold', y=0.98)

    # Top-left: Foot XZ
    ax1 = axes[0,0]
    ax1.set_title('① Foot Path (XZ)', fontsize=10, fontweight='bold')
    ax1.set_xlim(80,170); ax1.set_ylim(-185,-115)
    ax1.set_xlabel('X (mm)'); ax1.set_ylabel('Z (mm)'); ax1.grid(True)

    # Top-right: PID correction
    ax2 = axes[0,1]
    ax2.set_title('② PID Correction', fontsize=10, fontweight='bold')
    ax2.set_xlim(0, t[-1]); ax2.set_ylabel('Correction (deg)'); ax2.grid(True)
    pr = np.degrees(H['pr'][::ds])
    pp = np.degrees(H['pp'][::ds])
    mx = max(np.max(np.abs(pr)), np.max(np.abs(pp)), 0.1) * 1.3
    ax2.set_ylim(-mx, mx)

    # Bottom-left: Joint θ₂
    ax3 = axes[1,0]
    ax3.set_title('③ Joint θ₂: Offline vs Final', fontsize=10, fontweight='bold')
    ax3.set_xlim(0, t[-1]); ax3.set_ylabel('θ₂ (deg)'); ax3.grid(True)
    th2o = np.degrees(np.unwrap(H[f'{lg}_ot2'])[::ds])
    th2f = np.degrees(np.unwrap(H[f'{lg}_ft2'])[::ds])
    mx2 = max(np.max(np.abs(th2o)), np.max(np.abs(th2f)), 0.1) * 1.1
    ax3.set_ylim(np.min(th2o)-2, np.max(th2o)+2)

    # Bottom-right: Body roll result
    ax4 = axes[1,1]
    ax4.set_title('④ Result: Body Roll', fontsize=10, fontweight='bold')
    ax4.set_xlim(0, t[-1]); ax4.set_ylabel('Angle (deg)'); ax4.grid(True)
    br = np.degrees(H['br'][::ds])
    dr = np.degrees(H['dr'][::ds])
    mx3 = max(np.max(np.abs(br)), np.max(np.abs(dr)), 0.1) * 1.3
    ax4.set_ylim(-mx3, mx3)

    ox = H[f'{lg}_ox'][::ds]*1000; oz = H[f'{lg}_oz'][::ds]*1000
    ax_ = H[f'{lg}_ax'][::ds]*1000; az = H[f'{lg}_az'][::ds]*1000

    trail = 60
    l1a, = ax1.plot([],[], '-', color='#ffa502', lw=1.5, alpha=0.5)
    l1b, = ax1.plot([],[], '-', color='#54a0ff', lw=1.5, alpha=0.5)
    l1c, = ax1.plot([],[], 'o', color='#ffa502', ms=7, label='Offline')
    l1d, = ax1.plot([],[], 's', color='#54a0ff', ms=7, label='Adjusted')
    ax1.legend(fontsize=7)

    l2a, = ax2.plot([],[], color='#2ed573', lw=1.5, label='Roll')
    l2b, = ax2.plot([],[], color='#a29bfe', lw=1.5, label='Pitch')
    ax2.legend(fontsize=7)

    l3a, = ax3.plot([],[], color='#ffa502', lw=1.5, label='Offline')
    l3b, = ax3.plot([],[], color='#54a0ff', lw=1.5, ls='--', label='Final')
    ax3.legend(fontsize=7)

    l4a, = ax4.plot([],[], color='#ff6b6b', lw=1.5, label='Body roll')
    l4b, = ax4.plot([],[], '--', color='#e94560', lw=1, alpha=0.5, label='Disturbance')
    ax4.axhline(0, color='white', ls=':', alpha=0.2)
    ax4.legend(fontsize=7)

    def animate(i):
        s0 = max(0,i-trail)
        st, sf = slice(s0,i+1), slice(0,i+1)
        l1a.set_data(ox[st], oz[st]); l1b.set_data(ax_[st], az[st])
        l1c.set_data([ox[i]],[oz[i]]); l1d.set_data([ax_[i]],[az[i]])
        l2a.set_data(t[sf], pr[sf]); l2b.set_data(t[sf], pp[sf])
        l3a.set_data(t[sf], th2o[sf]); l3b.set_data(t[sf], th2f[sf])
        l4a.set_data(t[sf], br[sf]); l4b.set_data(t[sf], dr[sf])
        return ()

    ani = animation.FuncAnimation(fig, animate, frames=n, interval=80, blit=True)
    fig.tight_layout(rect=[0,0,1,0.95])
    path = os.path.join(OUT, 'full_pipeline_v2.gif')
    ani.save(path, writer='pillow', fps=12)
    plt.close(fig)
    print(f"  ✓ full_pipeline_v2.gif ({os.path.getsize(path)//1024} KB, {n} frames)")


# ── Main ─────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("Running simulation (T=3s, dt=5ms)...")
    H = run_sim(T_sec=3.0, dt=0.005)
    print(f"  Done: {len(H['t'])} steps")

    # Static plots (from main module — these don't need sim data)
    from sim_trajectory_combination import (plot_dshape_explained,
        plot_formula_explained, plot_pipeline_diagram)
    plot_dshape_explained(OUT)
    plot_formula_explained(OUT)
    plot_pipeline_diagram(OUT)

    # Generate combination static plot inline (matched keys)
    print("  Generating trajectory combination plot...")
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle("Trajectory Combination: Offline + PID = Final\n(Left-Front Leg)",
                 fontsize=12, fontweight='bold', y=0.99)
    t = H['t']; lg = 'left-front'

    ax = axes[0,0]
    ax.plot(t, H[f'{lg}_ox']*1000, color='#ffa502', lw=1.5, label='Offline')
    ax.plot(t, H[f'{lg}_ax']*1000, '--', color='#54a0ff', lw=1.5, label='Adjusted')
    ax.set_ylabel('Foot X (mm)'); ax.set_title('Foot X Position'); ax.legend(fontsize=8); ax.grid(True)

    ax = axes[0,1]
    ax.plot(t, H[f'{lg}_oz']*1000, color='#ffa502', lw=1.5, label='Offline')
    ax.plot(t, H[f'{lg}_az']*1000, '--', color='#54a0ff', lw=1.5, label='Adjusted')
    ax.set_ylabel('Foot Z (mm)'); ax.set_title('Foot Z Position'); ax.legend(fontsize=8); ax.grid(True)

    ax = axes[1,0]
    ax.plot(t, np.degrees(H['pr']), color='#2ed573', lw=1.5, label='Roll')
    ax.plot(t, np.degrees(H['pp']), color='#a29bfe', lw=1.5, label='Pitch')
    ax.set_ylabel('Correction (deg)'); ax.set_title('PID Output'); ax.legend(fontsize=8); ax.grid(True)
    ax.set_xlabel('Time (s)')

    ax = axes[1,1]
    ax.plot(t, np.degrees(H['dr']), '--', color='#e94560', lw=1, alpha=0.5, label='Disturbance')
    ax.plot(t, np.degrees(H['br']), color='#ff6b6b', lw=1.5, label='Body roll')
    ax.axhline(0, color='white', ls=':', alpha=0.2)
    ax.set_ylabel('Angle (deg)'); ax.set_title('Result: Body stays level')
    ax.legend(fontsize=8); ax.grid(True); ax.set_xlabel('Time (s)')

    fig.tight_layout(rect=[0,0,1,0.94])
    path = os.path.join(OUT, 'trajectory_combination_v2.png')
    fig.savefig(path, dpi=150, bbox_inches='tight', facecolor=fig.get_facecolor())
    plt.close(fig)
    print(f"  ✓ trajectory_combination_v2.png")

    # Animations (lightweight)
    anim_dshape(H)
    anim_pipeline(H)

    print(f"\n✅ All done! Results in: {OUT}/")
