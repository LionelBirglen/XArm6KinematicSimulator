"""
xarm6_fk.py
===========
Forward kinematics for the UFactory xArm 6.

Author : Lionel Birglen (with assistance from Anthropic Claude)
Polytechnique Montreal, 2026
"""

import math
from dataclasses import dataclass
import numpy as np
from xarm6_params import xarm6_params


@dataclass
class Pose:
    """End-effector pose in every orientation convention used by the UFactory SDK.

    Flange frame  (base → DH frame 6)
    ----------------------------------
    T          : ndarray (4,4)  homogeneous transform [mm]
    p          : ndarray (3,)   position [mm]
    rpy        : ndarray (3,)   [roll, pitch, yaw] rad — XYZ fixed-angle
                                R = Rz(yaw)·Ry(pitch)·Rx(roll)
    quat       : ndarray (4,)   [w, qx, qy, qz], w >= 0
    axis_angle : ndarray (3,)   [Rx, Ry, Rz] = phi·[nx,ny,nz], phi >= 0

    TCP frame  (base → TCP = flange · T_tcp from xarm6_params)
    -----------------------------------------------------------
    T_tcp          : ndarray (4,4)
    p_tcp          : ndarray (3,)
    rpy_tcp        : ndarray (3,)
    quat_tcp       : ndarray (4,)
    axis_angle_tcp : ndarray (3,)
    """
    T:              np.ndarray
    p:              np.ndarray
    rpy:            np.ndarray
    quat:           np.ndarray
    axis_angle:     np.ndarray
    T_tcp:          np.ndarray
    p_tcp:          np.ndarray
    rpy_tcp:        np.ndarray
    quat_tcp:       np.ndarray
    axis_angle_tcp: np.ndarray


def xarm6_fk(q: np.ndarray) -> Pose:
    """Compute the forward kinematics of the xArm 6.

    Parameters
    ----------
    q : array-like, shape (6,)
        Joint angles in radians.

    Returns
    -------
    pose : Pose
        Dataclass with flange and TCP pose fields in all orientation
        conventions.  Positions are in millimetres.

    Example
    -------
    >>> import numpy as np
    >>> from xarm6_fk import xarm6_fk
    >>> pose = xarm6_fk(np.deg2rad([30, -45, 0, 0, 45, -30]))
    >>> print(pose.p_ctp)
    >>> print(np.rad2deg(pose.rpy_tcp))
    """
    q = np.asarray(q, dtype=float).ravel()
    if q.size != 6:
        raise ValueError(f"q must have 6 elements, got {q.size}")

    DH, _, T_tcp_local = xarm6_params()[:3]

    # ---- Build flange transform T (base → flange) -----------------------
    T = np.eye(4)
    for i in range(6):
        T = T @ _sdh_transform(q[i] + DH[i, 0], DH[i, 1], DH[i, 2], DH[i, 3])

    # ---- TCP transform (base → TCP) -------------------------------------
    T_tcp_world = T @ T_tcp_local
    R_tcp = T_tcp_world[:3, :3]
    p_tcp = T_tcp_world[:3, 3]

    R = T[:3, :3]
    p = T[:3, 3]

    # ---- Orientation representations ------------------------------------
    rpy            = _rot2rpy(R)
    quat           = _rot2quat(R)
    axis_angle     = _quat2axisangle(quat)
    rpy_tcp        = _rot2rpy(R_tcp)
    quat_tcp       = _rot2quat(R_tcp)
    axis_angle_tcp = _quat2axisangle(quat_tcp)

    return Pose(
        T=T, p=p, rpy=rpy, quat=quat, axis_angle=axis_angle,
        T_tcp=T_tcp_world, p_tcp=p_tcp,
        rpy_tcp=rpy_tcp, quat_tcp=quat_tcp, axis_angle_tcp=axis_angle_tcp,
    )


# =========================================================================
#  Private helpers
# =========================================================================

def _sdh_transform(theta: float, d: float, a: float, alpha: float) -> np.ndarray:
    """Standard DH link matrix: Rz(theta)·Tz(d)·Tx(a)·Rx(alpha)."""
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([
        [ct, -st*ca,  st*sa,  a*ct],
        [st,  ct*ca, -ct*sa,  a*st],
        [0.,      sa,    ca,     d],
        [0.,      0.,    0.,    1.],
    ])


def _rot2rpy(R: np.ndarray) -> np.ndarray:
    """XYZ fixed-angle RPY from rotation matrix: R = Rz(yaw)·Ry(pitch)·Rx(roll)."""
    pitch = math.atan2(-R[2, 0], math.sqrt(R[0, 0]**2 + R[1, 0]**2))
    if abs(math.cos(pitch)) > 1e-10:
        roll = math.atan2( R[2, 1], R[2, 2])
        yaw  = math.atan2( R[1, 0], R[0, 0])
    else:                                           # gimbal lock
        roll = 0.0
        if pitch > 0:
            yaw = math.atan2( R[0, 1], R[1, 1])
        else:
            yaw = math.atan2(-R[0, 1], R[1, 1])
    return np.array([roll, pitch, yaw])


def _rot2quat(R: np.ndarray) -> np.ndarray:
    """Rotation matrix → unit quaternion [w, qx, qy, qz] using Shepperd's method."""
    trace = R[0,0] + R[1,1] + R[2,2]
    candidates = [trace, R[0,0], R[1,1], R[2,2]]
    idx = int(np.argmax(candidates))
    if idx == 0:
        w  = 0.5 * math.sqrt(1.0 + trace)
        s  = 0.25 / w
        qx = (R[2,1] - R[1,2]) * s
        qy = (R[0,2] - R[2,0]) * s
        qz = (R[1,0] - R[0,1]) * s
    elif idx == 1:
        qx = 0.5 * math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
        s  = 0.25 / qx
        w  = (R[2,1] - R[1,2]) * s
        qy = (R[0,1] + R[1,0]) * s
        qz = (R[0,2] + R[2,0]) * s
    elif idx == 2:
        qy = 0.5 * math.sqrt(1.0 - R[0,0] + R[1,1] - R[2,2])
        s  = 0.25 / qy
        w  = (R[0,2] - R[2,0]) * s
        qx = (R[0,1] + R[1,0]) * s
        qz = (R[1,2] + R[2,1]) * s
    else:
        qz = 0.5 * math.sqrt(1.0 - R[0,0] - R[1,1] + R[2,2])
        s  = 0.25 / qz
        w  = (R[1,0] - R[0,1]) * s
        qx = (R[0,2] + R[2,0]) * s
        qy = (R[1,2] + R[2,1]) * s
    q = np.array([w, qx, qy, qz])
    q /= np.linalg.norm(q)
    if q[0] < 0:
        q = -q
    return q


def _quat2axisangle(q: np.ndarray) -> np.ndarray:
    """Unit quaternion [w,qx,qy,qz] → UFactory axis-angle [Rx,Ry,Rz]=phi*axis."""
    w  = float(q[0])
    qv = q[1:4].copy()
    phi = 2.0 * math.acos(max(-1.0, min(1.0, w)))
    if phi > math.pi:
        phi = 2*math.pi - phi
        qv  = -qv
    sn = float(np.linalg.norm(qv))
    if sn < 1e-10:
        return np.zeros(3)
    return (phi / sn) * qv
