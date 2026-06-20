"""
xarm6_params.py
===============
Robot parameters for the UFactory xArm 6:
  - Standard DH table
  - Joint limits
  - TCP offset (flange → TCP)
  - TCP bounding box for visualisation

Author : Lionel Birglen (with assistance from Anthropic Claude)
Polytechnique Montreal, 2026
"""

import math
import numpy as np


def xarm6_params():
    """Return the Standard DH table, joint limits, TCP offset, and TCP
    bounding box for the UFactory xArm 6.

    Returns
    -------
    DH : ndarray, shape (6, 4)
        Each row = [theta_offset (rad), d (mm), a (mm), alpha (rad)].
        Actual joint angle for joint i:  theta_i = q[i] + DH[i, 0]

    qlim : ndarray, shape (6, 2)
        Joint limits [[lower, upper], ...] in radians.

    T_tcp : ndarray, shape (4, 4)
        Homogeneous transform from the flange (frame 6) to the TCP [mm].
        Edit TCP_translation and TCP_rotation below to match your tool.

    tcp_box : ndarray, shape (6,)
        Tool bounding box in the local TCP frame [mm]:
        [x_min, x_max, y_min, y_max, z_min, z_max].
        xarm6_plot draws the EE beam using this box.

    DH table (Standard DH, UFactory official, lengths in mm)
    ----------------------------------------------------------
    Joint  theta_offset      d       a        alpha
    -----  ---------------  ------  -------  ----------
      1       0              267.0    0        -pi/2
      2       T2_off           0     a2≈289.5   0
      3       T3_off           0     77.5      -pi/2
      4       0              342.5    0        +pi/2
      5       0                0     76.0      -pi/2
      6       0               97.0    0         0

    Joint limits
    ------------
    q1 : [-2π,    +2π  ]   ±360 deg
    q2 : [-2.042, +2.025]  -117 to +116 deg
    q3 : [-3.822, +0.174]  -219 deg to +10 deg
    q4 : [-2π,    +2π  ]
    q5 : [-1.692, +3.141]  -97 deg to +180 deg
    q6 : [-2π,    +2π  ]
    """
    # ---- DH table -------------------------------------------------------
    a2     = math.sqrt(284.5**2 + 53.5**2)   # ≈ 289.4887 mm
    T2_off = -math.atan2(284.5, 53.5)         # ≈ -1.3849179 rad
    T3_off = -T2_off

    #        theta_off   d       a      alpha
    DH = np.array([
        [0.0,       267.0,  0.0,    -math.pi/2],  # Joint 1
        [T2_off,      0.0,  a2,      0.0       ],  # Joint 2
        [T3_off,      0.0,  77.5,  -math.pi/2 ],  # Joint 3
        [0.0,       342.5,  0.0,    math.pi/2  ],  # Joint 4
        [0.0,         0.0,  76.0,  -math.pi/2 ],  # Joint 5
        [0.0,        97.0,  0.0,    0.0        ],  # Joint 6
    ])

    # ---- Joint limits ---------------------------------------------------
    #        lower        upper
    qlim = np.array([
        [-2*math.pi,  2*math.pi ],   # q1  ±360 deg
        [-2.059,      2.059     ],   # q2  ±118 deg
        [-3.927,      0.192     ],   # q3  -225 deg to +11 deg
        [-2*math.pi,  2*math.pi ],   # q4  ±360 deg
        [-1.692,      3.141     ],   # q5  -97 deg to +180 deg
        [-2*math.pi,  2*math.pi ],   # q6  ±360 deg
    ])

    # ---- TCP offset (flange → TCP) --------------------------------------
    # Edit the values below to match your physical tool.
    # TCP_translation : translation of the TCP origin in flange coords [mm]
    # TCP_rotation    : rotation of the TCP frame relative to the flange
    # Default: 50 mm along flange z-axis, no rotation.
    TCP_translation = np.array([0.0, 0.0, 50.0])   # mm
    TCP_rotation    = np.eye(3)                      # no rotation
    T_tcp = np.eye(4)
    T_tcp[:3, :3] = TCP_rotation
    T_tcp[:3,  3] = TCP_translation

    # ---- Tool bounding box in local TCP frame [mm] ----------------------
    # Format: [x_min, x_max, y_min, y_max, z_min, z_max]
    # The EE beam in xarm6_plot runs along the TCP z-axis from z_min to z_max,
    # with cross-section given by the x and y extents.
    tcp_box = np.array([-5.0, 5.0, -10.0, 10.0, -50.0, 0.0])   # mm

    return DH, qlim, T_tcp, tcp_box
