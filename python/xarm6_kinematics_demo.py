"""
xarm6_kinematics_demo.py
========================
Demonstration and validation of xarm6_fk and xarm6_ik.

Sections:
  1. FK  – display all orientation representations
  2. Round-trip: verify each representation reconstructs the rotation matrix
  3. IK  – all five input conventions
  4. Monte-Carlo accuracy test (100 random configurations)

Author : Lionel Birglen (with assistance from Anthropic Claude)
Polytechnique Montreal, 2026
"""

import math
import sys
import numpy as np
from xarm6_params import xarm6_params
from xarm6_fk import xarm6_fk
from xarm6_ik import xarm6_ik

# =========================================================================
# Shared test configuration
# =========================================================================
q_true = np.deg2rad([30, -45, 0, 0, 45, -30])
pose   = xarm6_fk(q_true)

# =========================================================================
# 1.  FK – all orientation representations
# =========================================================================
print('--- FK at q = [30, -45, 0, 0, 45, -30] deg ---')
print(f'Position    (mm)  : x={pose.p[0]:.3f}  y={pose.p[1]:.3f}  z={pose.p[2]:.3f}')
print(f'RPY         (rad) : roll={pose.rpy[0]:.6f}  pitch={pose.rpy[1]:.6f}  yaw={pose.rpy[2]:.6f}')
print(f'RPY         (deg) : roll={math.degrees(pose.rpy[0]):.4f}  pitch={math.degrees(pose.rpy[1]):.4f}  yaw={math.degrees(pose.rpy[2]):.4f}')
print(f'Quaternion        : w={pose.quat[0]:.6f}  qx={pose.quat[1]:.6f}  qy={pose.quat[2]:.6f}  qz={pose.quat[3]:.6f}')
print(f'Axis-angle  (rad) : Rx={pose.axis_angle[0]:.6f}  Ry={pose.axis_angle[1]:.6f}  Rz={pose.axis_angle[2]:.6f}')
phi = float(np.linalg.norm(pose.axis_angle))
if phi > 1e-10:
    axis = pose.axis_angle / phi
    print(f'  phi = {phi:.6f} rad ({math.degrees(phi):.4f} deg),  axis = {np.round(axis,4)}')
print()
print(f'TCP position (mm) : x={pose.p_tcp[0]:.3f}  y={pose.p_tcp[1]:.3f}  z={pose.p_tcp[2]:.3f}')
print()

# =========================================================================
# 2.  Round-trip validation
# =========================================================================
print('--- Orientation round-trip errors (Frobenius norm vs R from T) ---')
R_ref = pose.T[:3, :3]

def Rx(a): return np.array([[1,0,0],[0,math.cos(a),-math.sin(a)],[0,math.sin(a),math.cos(a)]])
def Ry(a): return np.array([[math.cos(a),0,math.sin(a)],[0,1,0],[-math.sin(a),0,math.cos(a)]])
def Rz(a): return np.array([[math.cos(a),-math.sin(a),0],[math.sin(a),math.cos(a),0],[0,0,1]])

roll, pitch, yaw = pose.rpy
R_rpy = Rz(yaw) @ Ry(pitch) @ Rx(roll)
print(f'RPY        -> R  error: {np.linalg.norm(R_ref - R_rpy, "fro"):.2e}')

w, qx, qy, qz = pose.quat
R_quat = np.array([
    [1-2*(qy**2+qz**2), 2*(qx*qy-w*qz),  2*(qx*qz+w*qy)],
    [2*(qx*qy+w*qz),  1-2*(qx**2+qz**2),  2*(qy*qz-w*qx)],
    [2*(qx*qz-w*qy),  2*(qy*qz+w*qx),  1-2*(qx**2+qy**2)],
])
print(f'Quaternion -> R  error: {np.linalg.norm(R_ref - R_quat, "fro"):.2e}')

rv  = pose.axis_angle
phi = float(np.linalg.norm(rv))
if phi > 1e-10:
    k   = rv / phi
    K   = np.array([[0,-k[2],k[1]],[k[2],0,-k[0]],[-k[1],k[0],0]])
    R_aa = np.eye(3) + math.sin(phi)*K + (1-math.cos(phi))*(K@K)
else:
    R_aa = np.eye(3)
print(f'Axis-angle -> R  error: {np.linalg.norm(R_ref - R_aa, "fro"):.2e}')
print()

# =========================================================================
# 3.  IK – all five input conventions
# =========================================================================
print('=== IK from all five input conventions ===\n')

def report_ik(label, Q, n, pose_true):
    print(f'[{label}]  solutions found: {n}')
    for k in range(n):
        p_k     = xarm6_fk(Q[k])
        pos_err = float(np.linalg.norm(p_k.p - pose_true.p))
        dR      = p_k.T[:3,:3].T @ pose_true.T[:3,:3]
        cos_a   = max(-1.0, min(1.0, (np.trace(dR)-1)/2))
        rot_err = math.degrees(math.acos(cos_a))
        q_deg   = np.round(np.degrees(Q[k]), 2)
        print(f'  Sol {k+1}: pos_err={pos_err:.3e} mm  rot_err={rot_err:.3e} deg  q={q_deg}')
    print()

Q1, n1 = xarm6_ik(pose.T, q_true)
report_ik('4x4 matrix', Q1, n1, pose)

Q2, n2 = xarm6_ik(pose, q_true)
report_ik('Pose dataclass (.T)', Q2, n2, pose)

Q3, n3 = xarm6_ik({'p': pose.p, 'rpy': pose.rpy}, q_true)
report_ik('position + RPY', Q3, n3, pose)

Q4, n4 = xarm6_ik({'p': pose.p, 'quat': pose.quat}, q_true)
report_ik('position + quaternion', Q4, n4, pose)

Q5, n5 = xarm6_ik({'p': pose.p, 'axis_angle': pose.axis_angle}, q_true)
report_ik('position + axis-angle', Q5, n5, pose)

if all(n > 0 for n in [n1, n3, n4, n5]):
    agree = max(np.linalg.norm(Q1[0]-Q3[0]),
                np.linalg.norm(Q1[0]-Q4[0]),
                np.linalg.norm(Q1[0]-Q5[0]))
    print(f'All-convention agreement (max diff in best solution): {agree:.2e} rad\n')

# =========================================================================
# 4.  Monte-Carlo round-trip test
# =========================================================================
print('--- Monte-Carlo round-trip test (100 random configs) ---')

_, qlim, _, _ = xarm6_params()
lo, hi = qlim[:, 0], qlim[:, 1]
rng = np.random.default_rng(42)

N_TEST   = 100
BAR_W    = 40
pos_errs = []
rot_errs = []
no_sol   = 0

print(f'  [{"–"*BAR_W}]   0%', end='', flush=True)

for t in range(1, N_TEST + 1):
    filled = round(BAR_W * t / N_TEST)
    bar    = '#' * filled + '-' * (BAR_W - filled)
    pct    = round(100 * t / N_TEST)
    print(f'\r  [{bar}] {pct:3d}%', end='', flush=True)

    q_rand = lo + rng.random(6) * (hi - lo)
    p_rand = xarm6_fk(q_rand)

    conv = (t - 1) % 4
    if   conv == 0: tgt = p_rand.T
    elif conv == 1: tgt = p_rand
    elif conv == 2: tgt = {'p': p_rand.p, 'rpy':  p_rand.rpy}
    else:           tgt = {'p': p_rand.p, 'quat': p_rand.quat}

    # NOTE: q_ref is intentionally NOT set to q_rand here. Passing the exact
    # answer as q_ref would make it the first seed, so Newton's method would
    # start already at the solution (zero error trivially). Omitting q_ref
    # tests genuine cold-start convergence from generic seeds only.
    Qt, ns = xarm6_ik(tgt, n_seeds=8)
    if ns == 0:
        no_sol += 1
        continue
    # Since q_ref was not provided, solutions are sorted by distance to
    # zeros(6); pick whichever returned solution is closest to q_rand
    # for a fair "did we find the right configuration" comparison.
    dists_to_true = np.sum((Qt - q_rand) ** 2, axis=1)
    best_idx = int(np.argmin(dists_to_true))
    p_rec = xarm6_fk(Qt[best_idx])
    pos_errs.append(float(np.linalg.norm(p_rec.p - p_rand.p)))
    dR    = p_rec.T[:3,:3].T @ p_rand.T[:3,:3]
    cos_a = max(-1.0, min(1.0, (np.trace(dR)-1)/2))
    rot_errs.append(math.degrees(math.acos(cos_a)))

print()   # newline after progress bar

pe = np.array(pos_errs)
re = np.array(rot_errs)
print(f'  Configs tested      : {N_TEST}')
print(f'  No solution found   : {no_sol}  ({100*no_sol/N_TEST:.1f}%)')
print(f'  Max position error  : {pe.max():.4e} mm')
print(f'  Mean position error : {pe.mean():.4e} mm')
print(f'  95th-pct pos error  : {np.percentile(pe, 95):.4e} mm')
print(f'  Max rotation error  : {re.max():.4e} deg')
print(f'  Mean rotation error : {re.mean():.4e} deg')
print('\nDone.')
