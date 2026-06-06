"""
xarm6_plot_demo.py
==================
Script to demonstrate xarm6_plot for static poses and a smooth animation.

Author : Lionel Birglen (with assistance from Anthropic Claude)
Polytechnique Montreal, 2026
"""

import math
import numpy as np
import matplotlib.pyplot as plt
from xarm6_params import xarm6_params
from xarm6_fk import xarm6_fk
from xarm6_ik import xarm6_ik
from xarm6_plot import xarm6_plot

# ---- Shared display settings --------------------------------------------
AXES_LIM = np.array([-600, 600, -500, 500, -80, 900])   # mm

# =========================================================================
# 1.  Static views of several named poses
# =========================================================================
poses = [
    (np.zeros(6),                                              'Home (all zeros)'),
    (np.deg2rad([ 0,  -60,   0,   0,   0,   0]),              'Arm Up'),
    (np.deg2rad([45,  -45,   0,   0,  45, -30]),              'Sample Pose A'),
    (np.deg2rad([-60, -30, -45,  20, -60,  90]),              'Sample Pose B'),
]

n = len(poses)
nrows = round(math.sqrt(n))
ncols = math.ceil(n / nrows)

fig_static = plt.figure(figsize=(5*ncols, 5*nrows))
fig_static.suptitle('xArm 6 – Static Configuration Viewer',
                    fontsize=13, fontweight='bold')

for k, (q_pose, label) in enumerate(poses):
    ax = fig_static.add_subplot(nrows, ncols, k+1, projection='3d')
    xarm6_plot(q_pose, ax=ax,
               show_frames=(k == 0),
               title=label,
               axes_lim=AXES_LIM)

plt.tight_layout()

# =========================================================================
# 2.  IK verification: target pose vs. recovered pose
# =========================================================================
print('\n--- IK round-trip visual check ---')
q_true    = np.deg2rad([30, -50, 0, 15, 40, -25])
pose_true = xarm6_fk(q_true)
Q_ik, n_sols = xarm6_ik(pose_true, q_true)
print(f'IK returned {n_sols} solution(s).')

if n_sols > 0:
    fig_ik = plt.figure(figsize=(12, 6))
    fig_ik.suptitle('xArm 6 – Forward vs Inverse Kinematics',
                    fontsize=13, fontweight='bold')

    ax1 = fig_ik.add_subplot(1, 2, 1, projection='3d')
    xarm6_plot(q_true, ax=ax1, show_frames=True,
               title='Target (FK of q_true)',
               axes_lim=AXES_LIM)

    ax2 = fig_ik.add_subplot(1, 2, 2, projection='3d')
    xarm6_plot(Q_ik[0], ax=ax2, show_frames=True,
               title=f'Best IK Solution (of {n_sols})',
               axes_lim=AXES_LIM)

    pose_rec = xarm6_fk(Q_ik[0])
    print(f'Position error of best IK solution: '
          f'{np.linalg.norm(pose_rec.p - pose_true.p):.4e} mm')
    plt.tight_layout()

# =========================================================================
# 3.  Smooth animation
# =========================================================================
answer = input('\nRun smooth animation? [y/N]: ').strip().lower()

if answer == 'y':
    _, qlim, _, _ = xarm6_params()
    lo, hi = qlim[:, 0], qlim[:, 1]

    N_FRAMES = 120
    t_vec    = np.linspace(0, 2 * math.pi, N_FRAMES)

    def _q_at(t):
        q = np.array([
            0.6 * math.sin(t),
            -0.5 + 0.3 * math.sin(2*t),
            min(0.0, -0.2 + 0.15 * math.cos(t)),
            0.4 * math.sin(3*t),
            0.5 + 0.3 * math.sin(t + 1),
            0.6 * math.cos(2*t),
        ])
        return np.clip(q, lo, hi)

    # Pre-compute TCP trajectory
    tcp_traj = np.array([xarm6_fk(_q_at(t)).p_tcp for t in t_vec]).T  # (3, N)

    TRAJ_COLOR = (0.85, 0.20, 0.20)

    fig_anim = plt.figure(figsize=(9, 8))
    ax_anim  = fig_anim.add_subplot(111, projection='3d')
    plt.ion()
    fig_anim.show()

    for k, t in enumerate(t_vec):
        q = _q_at(t)
        ax_anim.cla()
        xarm6_plot(q, ax=ax_anim,
                   show_frames=False,
                   axes_lim=AXES_LIM)

        # Draw TCP trajectory up to current frame
        ax_anim.plot(tcp_traj[0, :k+1], tcp_traj[1, :k+1], tcp_traj[2, :k+1],
                     '-', color=TRAJ_COLOR, linewidth=2.0)
        plt.pause(0.04)

    plt.ioff()
    print('Animation complete.')

plt.show()
print('\nDone.')
