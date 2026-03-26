"""
Enhanced Animation Generator
=============================
Creates multiple educational animations to help understand
the IMU PID posture stabilization controller.

Animations generated:
1. comparison_animation.gif   – Side-by-side: No Control vs PID
2. signal_flow_animation.gif  – Real-time signal propagation through all blocks
3. pid_terms_animation.gif    – P, I, D terms building up over time
4. robot_3d_animation.gif     – 3D wireframe robot body with 4 legs tilting
5. disturbance_types.gif      – All disturbance types compared

Usage:
    python3 generate_animations.py
"""

import os
import sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Circle
from matplotlib.gridspec import GridSpec

from robot_params import RobotParams
from pid_controller import PIDController, PIDGains
from imu_simulator import IMUSimulator
from body_posture import BodyPosture
from plant_model import PlantModel
from disturbance import Disturbance

# ── Style ─────────────────────────────────────────────────────────
plt.rcParams.update({
    'figure.facecolor': '#1a1a2e',
    'axes.facecolor':   '#16213e',
    'axes.edgecolor':   '#e94560',
    'axes.labelcolor':  '#eaeaea',
    'text.color':       '#eaeaea',
    'xtick.color':      '#aaaaaa',
    'ytick.color':      '#aaaaaa',
    'grid.color':       '#2a2a4a',
    'grid.alpha':       0.5,
    'legend.facecolor': '#1a1a2e',
    'legend.edgecolor': '#e94560',
    'font.size':        10,
})

COLORS = {
    'no_control': '#ff6b6b', 'pid':    '#54a0ff', 'roll':   '#e94560',
    'pitch':      '#0f3460', 'dist':   '#ffa502', 'setpt':  '#ffffff',
    'p_term':     '#ff6348', 'i_term': '#2ed573', 'd_term': '#1e90ff',
    'output':     '#ffd700', 'accent1':'#a29bfe', 'accent2':'#fd79a8',
    'accent3':    '#00cec9', 'accent4':'#fdcb6e', 'body':   '#54a0ff',
    'leg':        '#00cec9', 'foot':   '#ffd700', 'ground': '#666666',
}

OUTPUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")

# ══════════════════════════════════════════════════════════════════
#  Simulation helper — run a quick scenario
# ══════════════════════════════════════════════════════════════════

def run_quick_sim(enable_control=True, gains_kp=8.0, gains_ki=2.0, gains_kd=0.3,
                  dist_type="sinusoidal", T=8.0, dt=0.002):
    """Run a simulation and return downsampled results for animation."""
    params = RobotParams(dt=dt)
    gains = PIDGains(Kp=gains_kp, Ki=gains_ki, Kd=gains_kd)

    pid_r = PIDController(gains, dt, 0.2618, 0.5, "Roll")
    pid_p = PIDController(gains, dt, 0.2618, 0.5, "Pitch")
    imu = IMUSimulator(noise_std_roll=0.002, noise_std_pitch=0.002, dt=dt)
    posture = BodyPosture(params)
    plant = PlantModel(params)
    dist = Disturbance(dist_type=dist_type, roll_amplitude=np.radians(5),
                       pitch_amplitude=np.radians(3), roll_frequency=0.5,
                       pitch_frequency=0.3, step_time=3.0,
                       step_roll=np.radians(3), step_pitch=np.radians(2))

    n_steps = int(T / dt)
    for step in range(n_steps):
        t = step * dt
        dr, dp = dist.get(t)
        mr, mp = imu.measure(plant.roll, plant.pitch, t)
        if enable_control:
            cr = pid_r.compute(0.0, mr, t)
            cp = pid_p.compute(0.0, mp, t)
        else:
            cr, cp = 0.0, 0.0
            pid_r.compute(0.0, mr, t)
            pid_p.compute(0.0, mp, t)
        posture.compute_foot_adjustments(cr, cp, t)
        plant.step(cr, cp, dr, dp, t)

    return {
        'pid_roll': pid_r.get_history_arrays(),
        'pid_pitch': pid_p.get_history_arrays(),
        'imu': imu.get_history_arrays(),
        'posture': posture.get_history_arrays(),
        'plant': plant.get_history_arrays(),
        'disturbance': dist.get_history_arrays(),
    }


def save_anim(anim, fig, filename, n_frames, fps=15):
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    path = os.path.join(OUTPUT_DIR, filename)
    anim.save(path, writer='pillow', fps=fps,
              savefig_kwargs={'facecolor': fig.get_facecolor()})
    plt.close(fig)
    size_kb = os.path.getsize(path) / 1024
    print(f"  ✓ {filename} ({size_kb:.0f} KB, {n_frames} frames)")


# ══════════════════════════════════════════════════════════════════
#  Animation 1: Side-by-side Comparison (No Control vs PID)
# ══════════════════════════════════════════════════════════════════

def anim_comparison():
    print("  Generating comparison animation...")
    r_none = run_quick_sim(enable_control=False)
    r_pid  = run_quick_sim(enable_control=True)

    # Downsample
    ds = 20
    t = r_pid['plant']['time'][::ds]
    none_roll  = np.degrees(r_none['plant']['roll'][::ds])
    none_pitch = np.degrees(r_none['plant']['pitch'][::ds])
    pid_roll   = np.degrees(r_pid['plant']['roll'][::ds])
    pid_pitch  = np.degrees(r_pid['plant']['pitch'][::ds])
    dist_roll  = np.degrees(r_pid['disturbance']['roll'][::ds])
    dist_pitch = np.degrees(r_pid['disturbance']['pitch'][::ds])

    n_frames = len(t)
    body_w, body_l, nom_h = 0.222, 0.350, 0.170

    fig = plt.figure(figsize=(18, 10))
    gs = GridSpec(3, 2, figure=fig, height_ratios=[2, 2, 1.5], hspace=0.35, wspace=0.25)
    fig.suptitle("No Control vs PID — Side-by-Side Comparison",
                 fontsize=16, fontweight='bold', color='#ffffff', y=0.98)

    # Front views
    ax_nf = fig.add_subplot(gs[0, 0])
    ax_pf = fig.add_subplot(gs[0, 1])
    # Side views
    ax_ns = fig.add_subplot(gs[1, 0])
    ax_ps = fig.add_subplot(gs[1, 1])
    # Time traces
    ax_tr = fig.add_subplot(gs[2, 0])
    ax_tp = fig.add_subplot(gs[2, 1])

    for ax, title in [(ax_nf, 'NO CONTROL — Front View'),
                       (ax_pf, 'PID CONTROL — Front View'),
                       (ax_ns, 'NO CONTROL — Side View'),
                       (ax_ps, 'PID CONTROL — Side View')]:
        ax.set_xlim(-0.25, 0.25)
        ax.set_ylim(-0.03, 0.28)
        ax.set_aspect('equal')
        ax.set_title(title, fontsize=11, fontweight='bold')
        ax.grid(True)

    for ax in [ax_tr, ax_tp]:
        ax.set_xlim(0, t[-1])
        ax.grid(True)
    y_max = max(np.max(np.abs(none_roll)), np.max(np.abs(dist_roll))) * 1.2
    ax_tr.set_ylim(-y_max, y_max)
    ax_tr.set_title('Roll (°)', fontsize=11)
    ax_tr.set_xlabel('Time (s)')
    y_max_p = max(np.max(np.abs(none_pitch)), np.max(np.abs(dist_pitch))) * 1.2
    ax_tp.set_ylim(-y_max_p, y_max_p)
    ax_tp.set_title('Pitch (°)', fontsize=11)
    ax_tp.set_xlabel('Time (s)')

    # Artists
    def make_robot_artists(ax, color):
        body, = ax.plot([], [], 's-', color=color, linewidth=5, markersize=6)
        leg_l, = ax.plot([], [], '-o', color=COLORS['leg'], linewidth=2, markersize=4)
        leg_r, = ax.plot([], [], '-o', color=COLORS['leg'], linewidth=2, markersize=4)
        plat,  = ax.plot([], [], '-', color=COLORS['dist'], linewidth=3, alpha=0.5)
        gnd,   = ax.plot([-0.25, 0.25], [0, 0], '-', color=COLORS['ground'],
                         linewidth=1, alpha=0.3)
        return body, leg_l, leg_r, plat

    nb_f, nl_f, nr_f, np_f = make_robot_artists(ax_nf, COLORS['no_control'])
    pb_f, pl_f, pr_f, pp_f = make_robot_artists(ax_pf, COLORS['pid'])
    nb_s, nl_s, nr_s, np_s = make_robot_artists(ax_ns, COLORS['no_control'])
    pb_s, pl_s, pr_s, pp_s = make_robot_artists(ax_ps, COLORS['pid'])

    # Traces
    tr_none, = ax_tr.plot([], [], color=COLORS['no_control'], label='No Control', lw=1.5)
    tr_pid,  = ax_tr.plot([], [], color=COLORS['pid'], label='PID', lw=1.5)
    tr_dist, = ax_tr.plot([], [], '--', color=COLORS['dist'], label='Disturbance', lw=1, alpha=0.5)
    ax_tr.legend(loc='upper right', fontsize=8)

    tp_none, = ax_tp.plot([], [], color=COLORS['no_control'], label='No Control', lw=1.5)
    tp_pid,  = ax_tp.plot([], [], color=COLORS['pid'], label='PID', lw=1.5)
    tp_dist, = ax_tp.plot([], [], '--', color=COLORS['dist'], label='Disturbance', lw=1, alpha=0.5)
    ax_tp.legend(loc='upper right', fontsize=8)

    time_txt = fig.text(0.5, 0.01, '', ha='center', fontsize=12, color='#ffd700')

    def draw_front(body_art, leg_l_art, leg_r_art, plat_art, roll_rad, dist_r_rad):
        cr, sr = np.cos(roll_rad), np.sin(roll_rad)
        yl, zl = -body_w/2*cr, nom_h - body_w/2*sr
        yr, zr =  body_w/2*cr, nom_h + body_w/2*sr
        body_art.set_data([yl, yr], [zl, zr])
        leg_l_art.set_data([yl, yl], [zl, 0])
        leg_r_art.set_data([yr, yr], [zr, 0])
        cr2, sr2 = np.cos(dist_r_rad), np.sin(dist_r_rad)
        plat_art.set_data([-0.2*cr2, 0.2*cr2], [-0.2*sr2, 0.2*sr2])

    def draw_side(body_art, leg_l_art, leg_r_art, plat_art, pitch_rad, dist_p_rad):
        cp, sp = np.cos(pitch_rad), np.sin(pitch_rad)
        xb, zb = -body_l/2*cp, nom_h - body_l/2*sp
        xf, zf =  body_l/2*cp, nom_h + body_l/2*sp
        body_art.set_data([xb, xf], [zb, zf])
        leg_l_art.set_data([xb, xb], [zb, 0])
        leg_r_art.set_data([xf, xf], [zf, 0])
        cp2, sp2 = np.cos(dist_p_rad), np.sin(dist_p_rad)
        plat_art.set_data([-0.22*cp2, 0.22*cp2], [-0.22*sp2, 0.22*sp2])

    def animate(i):
        nr = np.radians(none_roll[i])
        pr_val = np.radians(pid_roll[i])
        dr = np.radians(dist_roll[i])
        npitch = np.radians(none_pitch[i])
        ppitch = np.radians(pid_pitch[i])
        dp = np.radians(dist_pitch[i])

        draw_front(nb_f, nl_f, nr_f, np_f, nr, dr)
        draw_front(pb_f, pl_f, pr_f, pp_f, pr_val, dr)
        draw_side(nb_s, nl_s, nr_s, np_s, npitch, dp)
        draw_side(pb_s, pl_s, pr_s, pp_s, ppitch, dp)

        tr_none.set_data(t[:i+1], none_roll[:i+1])
        tr_pid.set_data(t[:i+1], pid_roll[:i+1])
        tr_dist.set_data(t[:i+1], dist_roll[:i+1])
        tp_none.set_data(t[:i+1], none_pitch[:i+1])
        tp_pid.set_data(t[:i+1], pid_pitch[:i+1])
        tp_dist.set_data(t[:i+1], dist_pitch[:i+1])

        time_txt.set_text(f't = {t[i]:.2f}s   |   '
                          f'No Ctrl Roll: {none_roll[i]:+.2f}°   '
                          f'PID Roll: {pid_roll[i]:+.2f}°   |   '
                          f'No Ctrl Pitch: {none_pitch[i]:+.2f}°   '
                          f'PID Pitch: {pid_pitch[i]:+.2f}°')
        return ()

    anim = animation.FuncAnimation(fig, animate, frames=n_frames, interval=67, blit=True)
    fig.tight_layout(rect=[0, 0.03, 1, 0.96])
    save_anim(anim, fig, 'comparison_animation.gif', n_frames)


# ══════════════════════════════════════════════════════════════════
#  Animation 2: Signal Flow Through Control Blocks
# ══════════════════════════════════════════════════════════════════

def anim_signal_flow():
    print("  Generating signal flow animation...")
    r = run_quick_sim(enable_control=True)

    ds = 20
    t = r['plant']['time'][::ds]
    n_frames = len(t)

    # Extract signals
    dist_r   = np.degrees(r['disturbance']['roll'][::ds])
    meas_r   = np.degrees(r['imu']['filtered_roll'][::ds])
    err_r    = np.degrees(r['pid_roll']['error'][::ds])
    p_term_r = np.degrees(r['pid_roll']['p_term'][::ds])
    i_term_r = np.degrees(r['pid_roll']['i_term'][::ds])
    d_term_r = np.degrees(r['pid_roll']['d_term'][::ds])
    out_r    = np.degrees(r['pid_roll']['output'][::ds])
    body_r   = np.degrees(r['plant']['roll'][::ds])

    fig = plt.figure(figsize=(18, 12))
    fig.suptitle("Signal Flow Through Control Blocks — Roll Channel",
                 fontsize=16, fontweight='bold', color='#ffffff', y=0.98)

    # Create 7 subplots arranged as the signal path
    gs = GridSpec(4, 2, figure=fig, hspace=0.4, wspace=0.3)

    ax_dist = fig.add_subplot(gs[0, 0])
    ax_meas = fig.add_subplot(gs[0, 1])
    ax_err  = fig.add_subplot(gs[1, 0])
    ax_pid  = fig.add_subplot(gs[1, 1])
    ax_out  = fig.add_subplot(gs[2, 0])
    ax_body = fig.add_subplot(gs[2, 1])
    ax_cmp  = fig.add_subplot(gs[3, :])

    axes_info = [
        (ax_dist, 'BLOCK 1: Platform Disturbance', dist_r, COLORS['dist'],   '① dist'),
        (ax_meas, 'BLOCK 2: IMU Measurement',      meas_r, COLORS['accent2'],'② IMU'),
        (ax_err,  'BLOCK 3: Error = Setpoint - IMU',err_r,  COLORS['roll'],  '③ error'),
        (ax_pid,  'BLOCK 4: PID Terms',             None,   None,            None),
        (ax_out,  'BLOCK 5: PID Output (correction)',out_r,  COLORS['output'],'⑤ output'),
        (ax_body, 'BLOCK 6: Body Roll (result)',     body_r, COLORS['pid'],   '⑥ body'),
    ]

    lines = {}
    for ax, title, _, _, _ in axes_info:
        ax.set_title(title, fontsize=10, fontweight='bold')
        ax.set_xlim(0, t[-1])
        ax.grid(True)
        ax.set_xlabel('Time (s)', fontsize=8)

    # Set y-limits
    for ax, _, data, _, _ in axes_info:
        if data is not None:
            mx = np.max(np.abs(data)) * 1.3
            if mx > 0:
                ax.set_ylim(-mx, mx)

    # PID terms axis
    mx_pid = max(np.max(np.abs(p_term_r)), np.max(np.abs(i_term_r)),
                 np.max(np.abs(d_term_r))) * 1.3
    if mx_pid > 0:
        ax_pid.set_ylim(-mx_pid, mx_pid)

    # Create line artists
    lines['dist'], = ax_dist.plot([], [], color=COLORS['dist'], lw=1.5)
    lines['meas'], = ax_meas.plot([], [], color=COLORS['accent2'], lw=1.5)
    lines['err'],  = ax_err.plot([], [], color=COLORS['roll'], lw=1.5)
    lines['p'],    = ax_pid.plot([], [], color=COLORS['p_term'], lw=1.2, label='P')
    lines['i'],    = ax_pid.plot([], [], color=COLORS['i_term'], lw=1.2, label='I')
    lines['d'],    = ax_pid.plot([], [], color=COLORS['d_term'], lw=1.2, label='D')
    ax_pid.legend(loc='upper right', fontsize=8)
    lines['out'],  = ax_out.plot([], [], color=COLORS['output'], lw=1.5)
    lines['body'], = ax_body.plot([], [], color=COLORS['pid'], lw=1.5)

    # Comparison
    ax_cmp.set_title('Overall: Disturbance vs Body Response', fontsize=11, fontweight='bold')
    ax_cmp.set_xlim(0, t[-1])
    mx_cmp = max(np.max(np.abs(dist_r)), np.max(np.abs(body_r))) * 1.3
    ax_cmp.set_ylim(-mx_cmp, mx_cmp)
    ax_cmp.set_xlabel('Time (s)')
    ax_cmp.set_ylabel('Roll (°)')
    ax_cmp.grid(True)
    lines['cmp_dist'], = ax_cmp.plot([], [], '--', color=COLORS['dist'], lw=1, label='Disturbance', alpha=0.6)
    lines['cmp_body'], = ax_cmp.plot([], [], color=COLORS['pid'], lw=2, label='Body (PID)')
    ax_cmp.axhline(0, color=COLORS['setpt'], ls=':', alpha=0.3)
    ax_cmp.legend(loc='upper right', fontsize=9)

    time_txt = fig.text(0.5, 0.01, '', ha='center', fontsize=12, color='#ffd700')

    # Flow arrows (drawn as text)
    fig.text(0.50, 0.82, '→', ha='center', fontsize=24, color='#ffd700')
    fig.text(0.50, 0.60, '→', ha='center', fontsize=24, color='#ffd700')
    fig.text(0.50, 0.39, '→', ha='center', fontsize=24, color='#ffd700')

    def animate(i):
        s = slice(0, i+1)
        lines['dist'].set_data(t[s], dist_r[s])
        lines['meas'].set_data(t[s], meas_r[s])
        lines['err'].set_data(t[s], err_r[s])
        lines['p'].set_data(t[s], p_term_r[s])
        lines['i'].set_data(t[s], i_term_r[s])
        lines['d'].set_data(t[s], d_term_r[s])
        lines['out'].set_data(t[s], out_r[s])
        lines['body'].set_data(t[s], body_r[s])
        lines['cmp_dist'].set_data(t[s], dist_r[s])
        lines['cmp_body'].set_data(t[s], body_r[s])
        time_txt.set_text(f't = {t[i]:.2f}s   |   Disturbance: {dist_r[i]:+.2f}°  →  '
                          f'Error: {err_r[i]:+.2f}°  →  PID Out: {out_r[i]:+.2f}°  →  '
                          f'Body: {body_r[i]:+.2f}°')
        return ()

    anim = animation.FuncAnimation(fig, animate, frames=n_frames, interval=67, blit=True)
    fig.tight_layout(rect=[0, 0.03, 1, 0.96])
    save_anim(anim, fig, 'signal_flow_animation.gif', n_frames)


# ══════════════════════════════════════════════════════════════════
#  Animation 3: PID Terms Build-Up
# ══════════════════════════════════════════════════════════════════

def anim_pid_terms():
    print("  Generating PID terms animation...")
    r = run_quick_sim(enable_control=True)

    ds = 20
    t = r['plant']['time'][::ds]
    n_frames = len(t)

    err_r    = np.degrees(r['pid_roll']['error'][::ds])
    p_term   = np.degrees(r['pid_roll']['p_term'][::ds])
    i_term   = np.degrees(r['pid_roll']['i_term'][::ds])
    d_term   = np.degrees(r['pid_roll']['d_term'][::ds])
    output   = np.degrees(r['pid_roll']['output'][::ds])

    fig, axes = plt.subplots(5, 1, figsize=(14, 12), sharex=True)
    fig.suptitle("PID Controller — How Each Term Contributes (Roll)",
                 fontsize=16, fontweight='bold', color='#ffffff', y=0.98)

    configs = [
        (axes[0], 'Error Signal: e(t) = setpoint - measurement', err_r, COLORS['roll']),
        (axes[1], 'P Term: Kp × e(t) — Proportional (immediate response)', p_term, COLORS['p_term']),
        (axes[2], 'I Term: Ki × ∫e(t)dt — Integral (eliminates steady-state)', i_term, COLORS['i_term']),
        (axes[3], 'D Term: Kd × de/dt — Derivative (damps oscillation)', d_term, COLORS['d_term']),
        (axes[4], 'Output: u(t) = P + I + D — Combined correction', output, COLORS['output']),
    ]

    lines = []
    fills = []
    for ax, title, data, color in configs:
        ax.set_title(title, fontsize=10, fontweight='bold', loc='left')
        ax.set_xlim(0, t[-1])
        mx = np.max(np.abs(data)) * 1.3
        if mx > 0:
            ax.set_ylim(-mx, mx)
        ax.axhline(0, color='#ffffff', ls=':', alpha=0.2)
        ax.grid(True)
        line, = ax.plot([], [], color=color, lw=2)
        lines.append(line)
        # Pre-create fill (will update in animate)
        fill = ax.fill_between([], [], 0, alpha=0.2, color=color)
        fills.append(fill)

    axes[4].set_xlabel('Time (s)')
    time_txt = fig.text(0.5, 0.01, '', ha='center', fontsize=12, color='#ffd700')

    def animate(i):
        nonlocal fills
        s = slice(0, i+1)
        for j, (ax, _, data, color) in enumerate(configs):
            lines[j].set_data(t[s], data[s])
            fills[j].remove()
            fills[j] = ax.fill_between(t[s], data[s], 0, alpha=0.15, color=color)

        time_txt.set_text(f't = {t[i]:.2f}s   |   P: {p_term[i]:+.3f}°   '
                          f'I: {i_term[i]:+.3f}°   D: {d_term[i]:+.3f}°   '
                          f'→ Output: {output[i]:+.3f}°')
        return ()

    anim = animation.FuncAnimation(fig, animate, frames=n_frames, interval=67, blit=False)
    fig.tight_layout(rect=[0, 0.03, 1, 0.96])
    save_anim(anim, fig, 'pid_terms_animation.gif', n_frames)


# ══════════════════════════════════════════════════════════════════
#  Animation 4: 3D Robot Visualization
# ══════════════════════════════════════════════════════════════════

def anim_robot_3d():
    print("  Generating 3D robot animation...")
    r = run_quick_sim(enable_control=True)

    ds = 20
    t = r['plant']['time'][::ds]
    roll  = r['plant']['roll'][::ds]
    pitch = r['plant']['pitch'][::ds]
    dist_roll  = r['disturbance']['roll'][::ds]
    dist_pitch = r['disturbance']['pitch'][::ds]
    n_frames = len(t)

    # Robot geometry
    L, W, H = 0.175, 0.0965, 0.170  # half-lengths
    leg_len = 0.170

    # Body corners in body frame
    corners_body = np.array([
        [ L,  W, 0],  # front-left
        [ L, -W, 0],  # front-right
        [-L, -W, 0],  # back-right
        [-L,  W, 0],  # back-left
    ])

    fig = plt.figure(figsize=(16, 10))

    ax3d = fig.add_subplot(121, projection='3d')
    ax_traces = fig.add_subplot(122)

    fig.suptitle("3D Robot Body — Posture Stabilization on Swaying Platform",
                 fontsize=14, fontweight='bold', color='#ffffff', y=0.98)

    # 3D axis setup
    ax3d.set_xlim(-0.30, 0.30)
    ax3d.set_ylim(-0.30, 0.30)
    ax3d.set_zlim(-0.05, 0.35)
    ax3d.set_xlabel('X (m)', fontsize=9)
    ax3d.set_ylabel('Y (m)', fontsize=9)
    ax3d.set_zlabel('Z (m)', fontsize=9)
    ax3d.set_title('3D Wireframe View', fontsize=11)
    ax3d.set_facecolor('#16213e')
    ax3d.view_init(elev=25, azim=-60)

    # Traces
    ax_traces.set_xlim(0, t[-1])
    max_val = max(np.max(np.abs(np.degrees(roll))),
                  np.max(np.abs(np.degrees(pitch))),
                  np.max(np.abs(np.degrees(dist_roll))),
                  np.max(np.abs(np.degrees(dist_pitch)))) * 1.3
    ax_traces.set_ylim(-max_val, max_val)
    ax_traces.set_xlabel('Time (s)')
    ax_traces.set_ylabel('Angle (°)')
    ax_traces.set_title('Orientation History', fontsize=11)
    ax_traces.grid(True)
    ax_traces.axhline(0, color=COLORS['setpt'], ls=':', alpha=0.3)

    tr_roll, = ax_traces.plot([], [], color=COLORS['roll'], lw=1.5, label='Body Roll')
    tr_pitch, = ax_traces.plot([], [], color=COLORS['accent1'], lw=1.5, label='Body Pitch')
    tr_dr, = ax_traces.plot([], [], '--', color=COLORS['dist'], lw=1, alpha=0.5, label='Dist Roll')
    tr_dp, = ax_traces.plot([], [], ':', color=COLORS['dist'], lw=1, alpha=0.5, label='Dist Pitch')
    ax_traces.legend(loc='upper right', fontsize=8)

    time_txt = fig.text(0.5, 0.01, '', ha='center', fontsize=12, color='#ffd700')

    def rotation_matrix(roll_a, pitch_a):
        cr, sr = np.cos(roll_a), np.sin(roll_a)
        cp, sp = np.cos(pitch_a), np.sin(pitch_a)
        R = np.array([
            [cp,      sr*sp,   cr*sp],
            [0,       cr,     -sr   ],
            [-sp,     sr*cp,   cr*cp],
        ])
        return R

    def animate(i):
        ax3d.cla()
        ax3d.set_xlim(-0.30, 0.30)
        ax3d.set_ylim(-0.30, 0.30)
        ax3d.set_zlim(-0.05, 0.35)
        ax3d.set_xlabel('X', fontsize=8)
        ax3d.set_ylabel('Y', fontsize=8)
        ax3d.set_zlabel('Z', fontsize=8)
        ax3d.set_facecolor('#16213e')

        R = rotation_matrix(roll[i], pitch[i])
        center = np.array([0, 0, H])

        # Transform body corners
        corners_world = np.array([R @ c + center for c in corners_body])

        # Draw body (quadrilateral)
        body_loop = np.vstack([corners_world, corners_world[0:1]])
        ax3d.plot(body_loop[:, 0], body_loop[:, 1], body_loop[:, 2],
                  '-', color=COLORS['body'], linewidth=3)

        # Fill body surface
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection
        verts = [list(zip(corners_world[:, 0],
                          corners_world[:, 1],
                          corners_world[:, 2]))]
        body_poly = Poly3DCollection(verts, alpha=0.3, facecolor=COLORS['body'],
                                      edgecolor=COLORS['body'])
        ax3d.add_collection3d(body_poly)

        # Draw legs
        for ci, corner in enumerate(corners_world):
            foot = corner.copy()
            foot[2] = 0  # foot on ground
            ax3d.plot([corner[0], foot[0]],
                     [corner[1], foot[1]],
                     [corner[2], foot[2]],
                     '-o', color=COLORS['leg'], linewidth=2, markersize=4)

        # Draw ground plane
        gx = np.array([-0.25, 0.25, 0.25, -0.25, -0.25])
        gy = np.array([-0.25, -0.25, 0.25, 0.25, -0.25])
        gz = np.zeros(5)

        # Platform (tilted ground)
        R_plat = rotation_matrix(dist_roll[i], dist_pitch[i])
        plat_corners = np.array([
            [ 0.22,  0.18, 0],
            [ 0.22, -0.18, 0],
            [-0.22, -0.18, 0],
            [-0.22,  0.18, 0],
        ])
        plat_world = np.array([R_plat @ c for c in plat_corners])
        plat_loop = np.vstack([plat_world, plat_world[0:1]])
        ax3d.plot(plat_loop[:, 0], plat_loop[:, 1], plat_loop[:, 2],
                  '-', color=COLORS['dist'], linewidth=2, alpha=0.4)

        ax3d.set_title(f'Roll: {np.degrees(roll[i]):+.2f}°  Pitch: {np.degrees(pitch[i]):+.2f}°',
                       fontsize=10)

        # Update traces
        s = slice(0, i+1)
        tr_roll.set_data(t[s], np.degrees(roll[s]))
        tr_pitch.set_data(t[s], np.degrees(pitch[s]))
        tr_dr.set_data(t[s], np.degrees(dist_roll[s]))
        tr_dp.set_data(t[s], np.degrees(dist_pitch[s]))

        time_txt.set_text(f't = {t[i]:.2f}s   |   '
                          f'Body Roll: {np.degrees(roll[i]):+.2f}°   '
                          f'Body Pitch: {np.degrees(pitch[i]):+.2f}°   |   '
                          f'Platform Roll: {np.degrees(dist_roll[i]):+.2f}°   '
                          f'Platform Pitch: {np.degrees(dist_pitch[i]):+.2f}°')
        return ()

    anim = animation.FuncAnimation(fig, animate, frames=n_frames, interval=67, blit=False)
    fig.tight_layout(rect=[0, 0.03, 1, 0.96])
    save_anim(anim, fig, 'robot_3d_animation.gif', n_frames)


# ══════════════════════════════════════════════════════════════════
#  Animation 5: Controller Gain Effect (P vs PD vs PID)
# ══════════════════════════════════════════════════════════════════

def anim_gain_effect():
    print("  Generating gain effect animation...")
    r_p   = run_quick_sim(gains_kp=8.0, gains_ki=0.0, gains_kd=0.0)
    r_pd  = run_quick_sim(gains_kp=8.0, gains_ki=0.0, gains_kd=0.3)
    r_pid = run_quick_sim(gains_kp=8.0, gains_ki=2.0, gains_kd=0.3)

    ds = 20
    t = r_pid['plant']['time'][::ds]
    n_frames = len(t)

    p_roll   = np.degrees(r_p['plant']['roll'][::ds])
    pd_roll  = np.degrees(r_pd['plant']['roll'][::ds])
    pid_roll = np.degrees(r_pid['plant']['roll'][::ds])
    dist_r   = np.degrees(r_pid['disturbance']['roll'][::ds])

    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    fig.suptitle("Effect of Adding Controller Terms — Roll Channel",
                 fontsize=16, fontweight='bold', color='#ffffff', y=0.98)

    configs = [
        (axes[0], 'P Only (Kp=8):  Quick response but oscillation remains',
         p_roll, COLORS['p_term']),
        (axes[1], 'P + D (Kp=8, Kd=0.3):  D-term damps oscillation',
         pd_roll, COLORS['d_term']),
        (axes[2], 'P + I + D (Kp=8, Ki=2, Kd=0.3):  I-term removes steady-state error',
         pid_roll, COLORS['pid']),
    ]

    lines_ctrl = []
    lines_dist = []
    for ax, title, data, color in configs:
        ax.set_title(title, fontsize=10, fontweight='bold', loc='left')
        ax.set_xlim(0, t[-1])
        mx = max(np.max(np.abs(data)), np.max(np.abs(dist_r))) * 1.3
        ax.set_ylim(-mx, mx)
        ax.axhline(0, color=COLORS['setpt'], ls=':', alpha=0.3)
        ax.grid(True)
        ax.set_ylabel('Roll (°)')
        ld, = ax.plot([], [], '--', color=COLORS['dist'], lw=1, alpha=0.5, label='Disturbance')
        lc, = ax.plot([], [], color=color, lw=2, label='Body Roll')
        ax.legend(loc='upper right', fontsize=8)
        lines_ctrl.append(lc)
        lines_dist.append(ld)

    axes[2].set_xlabel('Time (s)')
    time_txt = fig.text(0.5, 0.01, '', ha='center', fontsize=12, color='#ffd700')

    def animate(i):
        s = slice(0, i+1)
        for j, (_, _, data, _) in enumerate(configs):
            lines_ctrl[j].set_data(t[s], data[s])
            lines_dist[j].set_data(t[s], dist_r[s])

        time_txt.set_text(f't = {t[i]:.2f}s   |   '
                          f'P: {p_roll[i]:+.2f}°   '
                          f'PD: {pd_roll[i]:+.2f}°   '
                          f'PID: {pid_roll[i]:+.2f}°')
        return ()

    anim = animation.FuncAnimation(fig, animate, frames=n_frames, interval=67, blit=True)
    fig.tight_layout(rect=[0, 0.03, 1, 0.96])
    save_anim(anim, fig, 'gain_effect_animation.gif', n_frames)


# ══════════════════════════════════════════════════════════════════
#  Main
# ══════════════════════════════════════════════════════════════════

def main():
    print("╔══════════════════════════════════════════════════════════╗")
    print("║   Generating Educational Animations                     ║")
    print("║   IMU PID Posture Stabilization                         ║")
    print("╚══════════════════════════════════════════════════════════╝")
    print()

    anim_comparison()
    anim_signal_flow()
    anim_pid_terms()
    anim_robot_3d()
    anim_gain_effect()

    print(f"\n✅ All 5 animations generated in: {OUTPUT_DIR}/")
    for f in sorted(os.listdir(OUTPUT_DIR)):
        if f.endswith('.gif'):
            size = os.path.getsize(os.path.join(OUTPUT_DIR, f)) / 1024
            print(f"  🎬 {f}  ({size:.0f} KB)")


if __name__ == "__main__":
    main()
