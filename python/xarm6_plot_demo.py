"""
xarm6_plot_demo.py
==================
Script to demonstrate xarm6_plot for static poses and a smooth animation.

Author : Lionel Birglen (with assistance from Anthropic Claude)
Polytechnique Montreal, 2026
"""

import math
import os
import shutil
import sys
import numpy as np
import matplotlib.pyplot as plt
from xarm6_params import xarm6_params
from xarm6_fk import xarm6_fk
from xarm6_ik import xarm6_ik
from xarm6_plot import xarm6_plot


def _find_ffmpeg():
    """Locate an ffmpeg executable even if it's not on the process PATH,
    and even if the running Python interpreter is unrelated to the conda
    installation where ffmpeg was actually installed (a common situation
    on systems with multiple Python installs: LibreOffice's bundled
    Python, a standalone Python install, and Anaconda all coexisting).

    Checks, in order:
      1. shutil.which('ffmpeg')        -- respects PATH as seen by Python
      2. <sys.prefix>/Library/bin/ffmpeg.exe   (conda, Windows, if this
                                                 Python IS the conda one)
      3. <sys.prefix>/bin/ffmpeg                (conda, Linux/macOS)
      4. CONDA_PREFIX env var + same two layouts, if set
      5. Common default conda/miniconda install locations under the
         user's home directory, searching both the base env and any
         named environments under envs/*/

    Returns the full path as a string, or None if not found.
    """
    found = shutil.which('ffmpeg')
    if found:
        return found

    search_roots = [sys.prefix]

    conda_prefix = os.environ.get('CONDA_PREFIX')
    if conda_prefix:
        search_roots.append(conda_prefix)

    home = os.path.expanduser('~')
    for base_name in ('anaconda3', 'miniconda3', 'Anaconda3', 'Miniconda3'):
        base_path = os.path.join(home, base_name)
        if os.path.isdir(base_path):
            search_roots.append(base_path)
            envs_dir = os.path.join(base_path, 'envs')
            if os.path.isdir(envs_dir):
                for env_name in os.listdir(envs_dir):
                    search_roots.append(os.path.join(envs_dir, env_name))

    for root in search_roots:
        for rel in (os.path.join('Library', 'bin', 'ffmpeg.exe'),
                    os.path.join('bin', 'ffmpeg')):
            candidate = os.path.join(root, rel)
            if os.path.isfile(candidate):
                return candidate

    return None

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
plt.show(block=False)
plt.pause(0.1)   # force the window to actually draw now, not at script end

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
    plt.show(block=False)
    plt.pause(0.1)

# =========================================================================
# 3.  Smooth animation
# =========================================================================
answer = input('\nRun smooth animation? [y/N]: ').strip().lower()

if answer == 'y':

    # ---- Optional: record the animation to an MP4 file ------------------
    rec_answer = input('Record animation to MP4 file? [y/N]: ').strip().lower()
    record_movie = (rec_answer == 'y')

    out_name = None
    if record_movie:
        out_name = input(
            'Output filename (without extension) [xarm6_animation]: '
        ).strip()
        if not out_name:
            out_name = 'xarm6_animation'

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
    print(f'Pre-computing TCP trajectory ({N_FRAMES} frames)...')
    tcp_traj = np.array([xarm6_fk(_q_at(t)).p_tcp for t in t_vec]).T  # (3, N)

    TRAJ_COLOR = (0.85, 0.20, 0.20)

    fig_anim = plt.figure(figsize=(9, 8))
    ax_anim  = fig_anim.add_subplot(111, projection='3d')

    def _draw_frame(k):
        t = t_vec[k]
        q = _q_at(t)
        ax_anim.cla()
        xarm6_plot(q, ax=ax_anim,
                   show_frames=False,
                   axes_lim=AXES_LIM)
        ax_anim.plot(tcp_traj[0, :k+1], tcp_traj[1, :k+1], tcp_traj[2, :k+1],
                     '-', color=TRAJ_COLOR, linewidth=2.0)

    if record_movie:
        # Saving to file: use FuncAnimation + FFMpegWriter. ffmpeg must be
        # locatable either on PATH or in the active conda environment.
        import matplotlib.animation as animation
        import matplotlib as mpl

        ffmpeg_path = _find_ffmpeg()
        if ffmpeg_path:
            mpl.rcParams['animation.ffmpeg_path'] = ffmpeg_path

        try:
            anim = animation.FuncAnimation(
                fig_anim, _draw_frame, frames=N_FRAMES, interval=40,
            )
            writer = animation.FFMpegWriter(fps=25, bitrate=2400)
            out_path = f'{out_name}.mp4'

            print(f'Encoding {N_FRAMES} frames to {out_path} ...')

            def _progress(current_frame, total_frames):
                pct = round(100 * (current_frame + 1) / total_frames)
                bar_w = 40
                filled = round(bar_w * (current_frame + 1) / total_frames)
                bar = '#' * filled + '-' * (bar_w - filled)
                print(f'\r  [{bar}] {pct:3d}%', end='', flush=True)

            anim.save(out_path, writer=writer, progress_callback=_progress)
            print()   # newline after progress bar
            print(f'Movie saved to {out_path}')
        except FileNotFoundError:
            print('ffmpeg could not be located automatically. This often '
                  'happens when multiple Python installations exist on the '
                  'same machine (e.g. a standalone Python, Anaconda, and '
                  'LibreOffice\'s bundled Python) and ffmpeg was installed '
                  'into a conda environment different from the Python '
                  'currently running this script. '
                  'Run "where ffmpeg" or check '
                  '"<conda_base>\\Library\\bin\\ffmpeg.exe" to find it, then '
                  'set it explicitly near the top of this script with:\n'
                  '  import matplotlib\n'
                  '  matplotlib.rcParams["animation.ffmpeg_path"] = '
                  'r"C:\\path\\to\\ffmpeg.exe"\n'
                  'Falling back to live playback instead.')
            plt.ion()
            fig_anim.show()
            for k in range(N_FRAMES):
                _draw_frame(k)
                plt.pause(0.04)
            plt.ioff()
    else:
        plt.ion()
        fig_anim.show()
        for k in range(N_FRAMES):
            _draw_frame(k)
            plt.pause(0.04)
        plt.ioff()

    print('Animation complete.')

plt.show()
print('\nDone.')
