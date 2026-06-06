"""
xarm6_ik.py
===========
Numerical inverse kinematics for the UFactory xArm 6.

Author : Lionel Birglen (with assistance from Anthropic Claude)
Polytechnique Montreal, 2026
"""

import math
from typing import Optional, Union
import numpy as np
from xarm6_params import xarm6_params
from xarm6_fk import xarm6_fk, Pose


def xarm6_ik(
    target,
    q_ref: Optional[np.ndarray] = None,
    *,
    n_seeds: int = 14,
    tol_pos: float = 0.05,
    tol_rot: float = 5e-4,
    max_iter: int = 300,
) -> tuple[np.ndarray, int]:
    """Numerical inverse kinematics of the UFactory xArm 6.

    Accepts the desired flange pose in any of the following formats:

    +-----------------------+------------------------------------------------+
    | Convention            | target argument                                |
    +-----------------------+------------------------------------------------+
    | 4×4 matrix            | ndarray (4,4) — base → flange [mm]             |
    | Pose dataclass        | Pose object returned by xarm6_fk (.T is used)  |
    | dict / p + rpy        | dict with 'p' (3,) mm and 'rpy' (3,) rad       |
    |                       |   rpy = [roll, pitch, yaw], R = Rz·Ry·Rx       |
    | dict / p + quat       | dict with 'p' (3,) mm and 'quat' (4,) [w,x,y,z]|
    | dict / p + axis_angle | dict with 'p' (3,) mm and 'axis_angle' (3,) rad |
    +-----------------------+------------------------------------------------+

    Uses a Jacobian damped-least-squares Newton solver with multiple random
    seeds. WHY NUMERICAL: the xArm 6 has a non-zero link offset on joint 5
    (a5 = 76 mm), so axes 4-5-6 do NOT form a true spherical wrist.

    Parameters
    ----------
    target : see table above
    q_ref : array-like (6,), optional
        Reference config used as first seed and for sorting output.
        Defaults to zeros(6).
    n_seeds : int
        Number of additional random seeds. Default 14.
    tol_pos : float
        Position convergence threshold [mm]. Default 0.05.
    tol_rot : float
        Rotation convergence threshold [rad]. Default 5e-4.
    max_iter : int
        Max Newton iterations per seed. Default 300.

    Returns
    -------
    Q : ndarray (M, 6)
        Valid solutions in radians, sorted by distance to q_ref.
        Shape (0, 6) if no solution found.
    n_solutions : int

    Examples
    --------
    >>> import numpy as np
    >>> from xarm6_fk import xarm6_fk
    >>> from xarm6_ik import xarm6_ik
    >>> pose = xarm6_fk(np.deg2rad([30, -45, 0, 0, 45, -30]))
    >>> Q, n = xarm6_ik(pose)                             # from Pose object
    >>> print(np.round(np.degrees(Q), 2));print(n)        # solutions in degrees, 2 d.p.
    >>> Q, n = xarm6_ik(pose.T)                           # from 4x4 matrix
    >>> print(np.round(np.degrees(Q), 2));print(n)        # solutions in degrees, 2 d.p.
    >>> Q, n = xarm6_ik({'p': pose.p, 'rpy': pose.rpy})   # from position+RPY
    >>> print(np.round(np.degrees(Q), 2));print(n)        # solutions in degrees, 2 d.p.
    """
    if q_ref is None:
        q_ref = np.zeros(6)
    q_ref = np.asarray(q_ref, dtype=float).ravel()

    T_des = _parse_target(target)

    _, qlim, _, _ = xarm6_params()
    lo = qlim[:, 0]
    hi = qlim[:, 1]

    # ---- Build seeds -----------------------------------------------------
    rng = np.random.default_rng(42)
    seeds = [q_ref.copy(), np.zeros(6)]
    seeds += [lo + rng.random(6) * (hi - lo) for _ in range(n_seeds)]

    # ---- Newton solver from each seed ------------------------------------
    solutions = []
    for s in seeds:
        q_sol, ok = _newton_ik(T_des, s, tol_pos, tol_rot, max_iter)
        if not ok:
            continue
        pose_chk = xarm6_fk(q_sol)
        if np.linalg.norm(pose_chk.p - T_des[:3, 3]) > tol_pos * 10:
            continue
        if np.all(q_sol >= lo) and np.all(q_sol <= hi):
            solutions.append(q_sol)

    if not solutions:
        return np.empty((0, 6)), 0

    Q = np.unique(np.round(np.array(solutions), 3), axis=0)
    dists = np.sum((Q - q_ref) ** 2, axis=1)
    Q = Q[np.argsort(dists)]
    return Q, Q.shape[0]


# =========================================================================
#  Input parsing — any convention → 4×4 T
# =========================================================================

def _parse_target(target) -> np.ndarray:
    """Convert any supported pose representation to a 4×4 matrix."""
    # 4×4 numpy array
    if isinstance(target, np.ndarray) and target.shape == (4, 4):
        return target.astype(float)

    # Pose dataclass (output of xarm6_fk)
    if isinstance(target, Pose):
        return target.T.astype(float)

    # dict-based conventions
    if isinstance(target, dict):
        if 'T' in target:
            return np.asarray(target['T'], dtype=float)
        if 'p' not in target:
            raise ValueError("target dict must contain key 'p' (position, 3-element).")
        p = np.asarray(target['p'], dtype=float).ravel()
        if p.size != 3:
            raise ValueError("'p' must be a 3-element vector.")
        if 'rpy' in target:
            rpy = np.asarray(target['rpy'], dtype=float).ravel()
            roll, pitch, yaw = rpy
            R = _Rz(yaw) @ _Ry(pitch) @ _Rx(roll)
        elif 'quat' in target:
            R = _quat2rot(np.asarray(target['quat'], dtype=float).ravel())
        elif 'axis_angle' in target:
            R = _axisangle2rot(np.asarray(target['axis_angle'], dtype=float).ravel())
        else:
            raise ValueError("target dict must contain 'rpy', 'quat', or 'axis_angle'.")
        T = np.eye(4)
        T[:3, :3] = R
        T[:3,  3] = p
        return T

    raise TypeError(
        "target must be a 4×4 ndarray, a Pose dataclass, or a dict with "
        "keys 'p' + ('rpy' | 'quat' | 'axis_angle')."
    )


# =========================================================================
#  Orientation helpers
# =========================================================================

def _Rx(a):
    return np.array([[1,0,0],[0,math.cos(a),-math.sin(a)],[0,math.sin(a),math.cos(a)]])

def _Ry(a):
    return np.array([[math.cos(a),0,math.sin(a)],[0,1,0],[-math.sin(a),0,math.cos(a)]])

def _Rz(a):
    return np.array([[math.cos(a),-math.sin(a),0],[math.sin(a),math.cos(a),0],[0,0,1]])

def _quat2rot(q: np.ndarray) -> np.ndarray:
    q = q / np.linalg.norm(q)
    w, qx, qy, qz = q
    return np.array([
        [1-2*(qy**2+qz**2),  2*(qx*qy-w*qz),   2*(qx*qz+w*qy)],
        [2*(qx*qy+w*qz),   1-2*(qx**2+qz**2),   2*(qy*qz-w*qx)],
        [2*(qx*qz-w*qy),   2*(qy*qz+w*qx),   1-2*(qx**2+qy**2)],
    ])

def _axisangle2rot(rv: np.ndarray) -> np.ndarray:
    phi = float(np.linalg.norm(rv))
    if phi < 1e-10:
        return np.eye(3)
    k = rv / phi
    K = np.array([[0,-k[2],k[1]],[k[2],0,-k[0]],[-k[1],k[0],0]])
    return np.eye(3) + math.sin(phi)*K + (1-math.cos(phi))*(K@K)


# =========================================================================
#  Newton IK — single seed
# =========================================================================

def _newton_ik(
    T_des: np.ndarray,
    q0: np.ndarray,
    tol_pos: float,
    tol_rot: float,
    max_iter: int,
    W_ROT: float = 100.0,
    LAMBDA: float = 1e-4,
    MAX_STEP: float = 0.12,
) -> tuple[np.ndarray, bool]:
    """Damped-least-squares Newton IK from a single seed, unclamped."""
    q = q0.astype(float).copy()

    for _ in range(max_iter):
        pose = xarm6_fk(q)
        Tc   = pose.T

        dp = T_des[:3, 3] - Tc[:3, 3]                     # (3,) mm
        dR = T_des[:3, :3] @ Tc[:3, :3].T
        dw = 0.5 * np.array([dR[2,1]-dR[1,2],
                              dR[0,2]-dR[2,0],
                              dR[1,0]-dR[0,1]])            # (3,) rad

        if np.linalg.norm(dp) < tol_pos and np.linalg.norm(dw) < tol_rot:
            return q, True

        Jp, Jr = _numerical_jacobian(q, Tc)

        err = np.concatenate([dp, W_ROT * dw])             # (6,)
        J   = np.vstack([Jp, W_ROT * Jr])                  # (6,6)
        dq  = np.linalg.solve(J.T @ J + LAMBDA * np.eye(6), J.T @ err)

        scale = min(1.0, MAX_STEP / max(float(np.abs(dq).max()), 1e-9))
        q = q + scale * dq

    # Final check
    pose = xarm6_fk(q)
    Tc   = pose.T
    dp   = T_des[:3, 3] - Tc[:3, 3]
    dR   = T_des[:3, :3] @ Tc[:3, :3].T
    dw   = 0.5 * np.array([dR[2,1]-dR[1,2], dR[0,2]-dR[2,0], dR[1,0]-dR[0,1]])
    return q, (np.linalg.norm(dp) < tol_pos and np.linalg.norm(dw) < tol_rot)


def _numerical_jacobian(q: np.ndarray, T0: np.ndarray, eps: float = 1e-6):
    """Forward-difference numerical Jacobian, returns (Jp (3,6), Jr (3,6))."""
    p0 = T0[:3, 3]
    Jp = np.zeros((3, 6))
    Jr = np.zeros((3, 6))
    for i in range(6):
        dq    = np.zeros(6)
        dq[i] = eps
        T1    = xarm6_fk(q + dq).T
        Jp[:, i] = (T1[:3, 3] - p0) / eps
        dR       = T1[:3, :3] @ T0[:3, :3].T
        Jr[:, i] = np.array([dR[2,1]-dR[1,2],
                              dR[0,2]-dR[2,0],
                              dR[1,0]-dR[0,1]]) / (2 * eps)
    return Jp, Jr
