#!/usr/bin/env python3
"""
Flight plots — Jun 7 NBV inspection sessions.

Generates plots for each configured session:
  plots/flight_3d_<tag>.png
  plots/flight_xy_<tag>.png
  plots/flight_zy_<tag>.png
  plots/flight_confidence_<tag>.png

Also prints best photo (max confidence) per waypoint for each session.
"""

import re
import os
from pathlib import Path

HOME = Path.home()

SESSIONS = {
    '1aruco': {
        'offboard_log': HOME / '.ros/log/python3_1504_1780840911873.log',
        'nbv_log':      HOME / '.ros/log/python3_1506_1780840911873.log',
        'photo_dir':    HOME / 'ros2_ws_AA/aruco_output/Test_Accomplished_onearuco',
        't0_unix':      1780840912.089,
        't0_wallsec':   16 * 3600 + 1 * 60 + 51,   # 57711 — 16:01:51
        'conf_thresh':  0.70,
        'title_suffix': 'One ArUco marker (ID 0)',
    },
    '2aruco': {
        'offboard_log': HOME / '.ros/log/python3_1510_1780843001642.log',
        'nbv_log':      HOME / '.ros/log/python3_1512_1780843001642.log',
        'photo_dir':    HOME / 'ros2_ws_AA/aruco_output/Test_accomplished_2aruco',
        't0_unix':      1780843001.844,
        't0_wallsec':   16 * 3600 + 36 * 60 + 41,  # 59801 — 16:36:41
        'conf_thresh':  0.50,
        'title_suffix': 'Two ArUco markers (ID 0 and ID 1)',
    },
}

RE_OFF_POS = re.compile(r'\[(\d+\.\d+)\].*Pos  curr=\(([+-]?\d+\.\d+), ([+-]?\d+\.\d+), ([+-]?\d+\.\d+)\)')
RE_OFF_WP  = re.compile(r'\[(\d+\.\d+)\].*Target: x=([+-]?\d+\.\d+), y=([+-]?\d+\.\d+), z=([+-]?\d+\.\d+)')
RE_NBV_WP  = re.compile(r'\[(\d+\.\d+)\].*NBV: published goal -> x=([+-]?\d+\.\d+), y=([+-]?\d+\.\d+), z=([+-]?\d+\.\d+)')
RE_NBV_POS = re.compile(r'\[(\d+\.\d+)\].*NBV LOOP \| odom=\(([+-]?\d+\.\d+), ([+-]?\d+\.\d+), ([+-]?\d+\.\d+)\)')
RE_CONF    = re.compile(r'\[(\d+\.\d+)\].*Target (\d+) \|.*conf=(\d+\.\d+)')
RE_DONE    = re.compile(r'\[(\d+\.\d+)\].*Mission complete\.')


# ---------------------------------------------------------------------------
# Parsing
# ---------------------------------------------------------------------------

def parse(cfg):
    off_pos, off_wps = [], []
    with open(cfg['offboard_log'], errors='replace') as f:
        for line in f:
            m = RE_OFF_POS.search(line)
            if m:
                off_pos.append((float(m[1]), float(m[2]), float(m[3]), float(m[4])))
                continue
            m = RE_OFF_WP.search(line)
            if m:
                off_wps.append((float(m[1]), float(m[2]), float(m[3]), float(m[4])))

    nbv_wps, nbv_pos, conf_by_id, t_complete = [], [], {}, None
    with open(cfg['nbv_log'], errors='replace') as f:
        for line in f:
            m = RE_NBV_WP.search(line)
            if m:
                nbv_wps.append((float(m[1]), float(m[2]), float(m[3]), float(m[4])))
                continue
            m = RE_NBV_POS.search(line)
            if m:
                nbv_pos.append((float(m[1]), float(m[2]), float(m[3]), float(m[4])))
                continue
            m = RE_CONF.search(line)
            if m:
                conf_by_id.setdefault(int(m[2]), []).append((float(m[1]), float(m[3])))
                continue
            m = RE_DONE.search(line)
            if m:
                t_complete = float(m[1])

    return off_pos, off_wps, nbv_wps, nbv_pos, conf_by_id, t_complete


def build_traj(off_pos, nbv_pos, t_complete):
    """Merge offboard odom + NBV LOOP odom, cut at mission complete."""
    t0 = off_pos[0][0]
    t_off_end = off_pos[-1][0]
    t_cut = t_complete if t_complete else t_off_end

    raw = list(off_pos)
    for p in nbv_pos:
        if p[0] > t_off_end:
            raw.append(p)
    raw.sort(key=lambda p: p[0])

    traj = [(p[0] - t0, p[1], p[2], -p[3])   # (t_rel, x, y, alt)
            for p in raw if p[0] <= t_cut + 1.0]
    return traj, t0


# ---------------------------------------------------------------------------
# Photo filtering
# ---------------------------------------------------------------------------

def hhmmss_to_unix(hhmmss_str, t0_unix, t0_wallsec):
    h = int(hhmmss_str[0:2])
    m = int(hhmmss_str[2:4])
    s = int(hhmmss_str[4:6])
    wall_sec = h * 3600 + m * 60 + s
    return t0_unix + (wall_sec - t0_wallsec)


def nearest_conf(conf_by_id, t_unix):
    total = 0.0
    for mid, samples in conf_by_id.items():
        best = min(samples, key=lambda s: abs(s[0] - t_unix))
        total += best[1]
    return total


def select_best_photos(off_wps, nbv_wps, conf_by_id, cfg):
    all_wps  = [off_wps[0]] + list(nbv_wps)
    wp_unix  = [w[0] for w in all_wps]
    last_t   = max(s[0] for slist in conf_by_id.values() for s in slist)
    wp_unix.append(last_t + 10)

    photos = []
    photo_dir = cfg['photo_dir']
    if photo_dir.exists():
        for fname in sorted(os.listdir(photo_dir)):
            if not fname.endswith('.jpg'):
                continue
            hhmmss = fname[-10:-4]
            try:
                t_unix = hhmmss_to_unix(hhmmss, cfg['t0_unix'], cfg['t0_wallsec'])
            except Exception:
                continue
            photos.append((t_unix, fname))

    results = []
    for i in range(len(wp_unix) - 1):
        t_start = wp_unix[i]
        t_end   = wp_unix[i + 1]
        label   = f'W{i}'

        group = [(t, f) for t, f in photos if t_start <= t < t_end]
        if not group:
            continue

        best_f, best_conf = None, -1.0
        for t_ph, fname in group:
            c = nearest_conf(conf_by_id, t_ph)
            if c > best_conf:
                best_conf = c
                best_f    = fname

        results.append((label, best_f, best_conf))

    return results


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

def plot_session(tag, cfg, out_dir):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    off_pos, off_wps, nbv_wps, nbv_pos, conf_by_id, t_complete = parse(cfg)
    traj, t0_abs = build_traj(off_pos, nbv_pos, t_complete)

    tx  = np.array([p[1] for p in traj])
    ty  = np.array([p[2] for p in traj])
    ta  = np.array([p[3] for p in traj])

    all_wps = [off_wps[0]] + list(nbv_wps)
    wx   = np.array([w[1] for w in all_wps])
    wy   = np.array([w[2] for w in all_wps])
    walt = np.array([-w[3] for w in all_wps])

    mc_x, mc_y, mc_a = tx[-1], ty[-1], ta[-1]

    TRAJ = '#1f77b4'
    WP   = '#d62728'
    MC   = '#2ca02c'
    suffix = cfg['title_suffix']

    # ---- 3D ----
    fig1 = plt.figure(figsize=(9, 7))
    ax3  = fig1.add_subplot(111, projection='3d')
    ax3.plot(tx, ty, ta, color=TRAJ, lw=2.0, zorder=3, label='Flight path')
    ax3.scatter(wx, wy, walt, color=WP, s=55, marker='s', zorder=5,
                depthshade=False, edgecolors='darkred', lw=0.7, label='Waypoints')
    for i, (x_, y_, z_) in enumerate(zip(wx, wy, walt)):
        ax3.text(x_ + 0.05, y_, z_ + 0.06, f'W{i}', fontsize=7, color=WP)
    ax3.scatter([mc_x], [mc_y], [mc_a], color=MC, s=160, marker='s', zorder=7,
                depthshade=False, edgecolors='darkgreen', lw=2.0,
                label=f'Mission complete (alt={mc_a:.2f} m)')
    ax3.set_xlabel('X (m)', labelpad=8)
    ax3.set_ylabel('Y (m)', labelpad=8)
    ax3.set_zlabel('Z (m)', labelpad=8)
    ax3.set_title(f'NBV Inspection Trajectory\nArUco 4×4 — {suffix}')
    ax3.legend(loc='upper left', fontsize=9)
    ax3.view_init(elev=25, azim=-55)
    fig1.tight_layout()
    fig1.savefig(out_dir / f'flight_3d_{tag}.png', bbox_inches='tight')
    plt.close(fig1)
    print(f'Saved: flight_3d_{tag}.png')

    # ---- XY ----
    fig2, ax2 = plt.subplots(figsize=(7, 6))
    ax2.plot(tx, ty, color=TRAJ, lw=2.0, zorder=3, label='Flight path')
    ax2.scatter(wx, wy, color=WP, s=55, marker='s', zorder=5,
                edgecolors='darkred', lw=0.7, label='Waypoints')
    for i, (x_, y_) in enumerate(zip(wx, wy)):
        ax2.annotate(f'W{i}', (x_, y_), textcoords='offset points',
                     xytext=(4, 4), fontsize=7, color=WP)
    ax2.scatter([mc_x], [mc_y], color=MC, s=160, marker='s', zorder=7,
                edgecolors='darkgreen', lw=2.0,
                label=f'Mission complete (alt={mc_a:.2f} m)')
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_title(f'Top-down View — {suffix}')
    ax2.set_aspect('equal')
    ax2.legend(loc='best', fontsize=9)
    ax2.grid(True, alpha=0.25)
    fig2.tight_layout()
    fig2.savefig(out_dir / f'flight_xy_{tag}.png', bbox_inches='tight')
    plt.close(fig2)
    print(f'Saved: flight_xy_{tag}.png')

    # ---- Altitude vs Y ----
    fig3, ax3z = plt.subplots(figsize=(7, 5))
    ax3z.plot(ty, ta, color=TRAJ, lw=2.0, zorder=3, label='Flight path')
    ax3z.scatter(wy, walt, color=WP, s=55, marker='s', zorder=5,
                 edgecolors='darkred', lw=0.7, label='Waypoints')
    for i, (y_, z_) in enumerate(zip(wy, walt)):
        ax3z.annotate(f'W{i}', (y_, z_), textcoords='offset points',
                      xytext=(4, 4), fontsize=7, color=WP)
    ax3z.scatter([mc_y], [mc_a], color=MC, s=160, marker='s', zorder=7,
                 edgecolors='darkgreen', lw=2.0,
                 label=f'Mission complete (alt={mc_a:.2f} m)')
    ax3z.set_xlabel('Y (m)')
    ax3z.set_ylabel('Altitude (m)')
    ax3z.set_title(f'Side View — Altitude vs Y — {suffix}')
    ax3z.set_ylim(bottom=0)
    ax3z.legend(loc='best', fontsize=9)
    ax3z.grid(True, alpha=0.25)
    fig3.tight_layout()
    fig3.savefig(out_dir / f'flight_zy_{tag}.png', bbox_inches='tight')
    plt.close(fig3)
    print(f'Saved: flight_zy_{tag}.png')

    # ---- Confidence ----
    fig4, ax4 = plt.subplots(figsize=(10, 5))
    COLORS = {0: '#1f77b4', 1: '#d62728'}
    for mid, samples in sorted(conf_by_id.items()):
        ct = np.array([s[0] - t0_abs for s in samples])
        cv = np.array([s[1] for s in samples])
        ax4.plot(ct, cv, '-o', color=COLORS.get(mid, 'grey'),
                 lw=2.0, ms=4, label=f'Marker ID {mid}')
    thresh = cfg['conf_thresh']
    ax4.axhline(thresh, color='#ff7f0e', lw=2.0, ls='--',
                label=f'Threshold {thresh:.2f}')
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Detection confidence')
    ax4.set_ylim(bottom=0.20)
    ax4.set_title(f'ArUco Detection Confidence — {suffix}')
    ax4.legend(loc='upper left')
    ax4.grid(True, alpha=0.25)
    fig4.tight_layout()
    fig4.savefig(out_dir / f'flight_confidence_{tag}.png', bbox_inches='tight')
    plt.close(fig4)
    print(f'Saved: flight_confidence_{tag}.png')

    # ---- Best photos ----
    print(f'\n--- Best photo per waypoint [{tag}] ---')
    best = select_best_photos(off_wps, nbv_wps, conf_by_id, cfg)
    for label, fname, conf in best:
        print(f'  {label:6s}  {fname}  (conf_sum={conf:.3f})')
    print(f'Mission complete: X={mc_x:.2f}  Y={mc_y:.2f}  alt={mc_a:.2f} m\n')


def main():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    plt.rcParams.update({
        'font.family': 'DejaVu Sans',
        'font.size': 11,
        'axes.labelsize': 12,
        'axes.titlesize': 13,
        'legend.fontsize': 10,
        'figure.dpi': 150,
    })

    out_dir = Path(__file__).parent.parent / 'plots'
    out_dir.mkdir(exist_ok=True)

    for tag, cfg in SESSIONS.items():
        print(f'\n=== Session: {tag} ===')
        plot_session(tag, cfg, out_dir)


if __name__ == '__main__':
    main()
