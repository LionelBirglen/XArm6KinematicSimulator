"""
xarm6_plot.py
=============
3-D visualisation of the UFactory xArm 6 robotic arm.

Author : Lionel Birglen (with assistance from Anthropic Claude)
Polytechnique Montreal, 2026
"""

import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from xarm6_params import xarm6_params
from xarm6_fk import xarm6_fk, _sdh_transform


def xarm6_plot(
    q: np.ndarray,
    *,
    ax: "Axes3D | None" = None,
    show_frames: bool = False,
    show_flange: bool = False,
    link_color:  tuple = (0.25, 0.52, 0.80),
    joint_color: tuple = (0.90, 0.35, 0.20),
    frame_scale: float = 60.0,
    axes_lim: "np.ndarray | None" = None,
    title: "str | None" = None,
    figsize: tuple = (9, 8),
) -> plt.Figure:
    """Draw a 3-D model of the xArm 6 in joint configuration q.

    Parameters
    ----------
    q : array-like (6,)
        Joint angles in radians.
    ax : Axes3D, optional
        Existing axes to draw into (new figure if omitted).
    show_frames : bool
        Draw coordinate frames at every joint. Default False.
    show_flange : bool
        Draw the flange frame triad. Default False (only TCP frame shown).
    link_color : RGB tuple
        Link beam colour. Default blue.
    joint_color : RGB tuple
        Joint cylinder colour. Default red-orange.
    frame_scale : float
        Length of frame arrows [mm]. Default 60.
    axes_lim : array-like (6,), optional
        [xmin, xmax, ymin, ymax, zmin, zmax] in mm.
        Auto-computed from joint origins if omitted.
    title : str, optional
        Figure title. Auto-generated from q if omitted.
    figsize : tuple
        Figure size. Default (9, 8).

    Returns
    -------
    fig : matplotlib Figure

    Examples
    --------
    >>> import numpy as np
    >>> import matplotlib.pyplot as plt
    >>> from xarm6_plot import xarm6_plot
    >>>
    >>> # Simple static plot at home pose
    >>> fig = xarm6_plot(np.zeros(6))
    >>> plt.show()
    >>>
    >>> # Custom pose with intermediate frames visible
    >>> fig = xarm6_plot(np.deg2rad([30, -45, 0, 0, 45, -30]), show_frames=True)
    >>> plt.show()
    >>>
    >>> # Fixed axis limits
    >>> fig = xarm6_plot(
    ...     np.deg2rad([45, -60, 0, 0, 30, 0]),
    ...     axes_lim=np.array([-600, 600, -500, 500, -80, 900]),
    ... )
    >>> plt.show()
    >>>
    >>> # Animation: reuse a single axes object for performance
    >>> fig = plt.figure()
    >>> ax = fig.add_subplot(111, projection='3d')
    >>> for t in np.linspace(0, 2 * np.pi, 120):
    ...     q = np.array([0.6*np.sin(t), -0.5+0.3*np.sin(2*t), 0,
    ...                   0.4*np.sin(3*t), 0.5, 0.3*np.cos(t)])
    ...     xarm6_plot(q, ax=ax)
    ...     plt.pause(0.04)
    """
    q = np.asarray(q, dtype=float).ravel()
    if q.size != 6:
        raise ValueError(f"q must have 6 elements, got {q.size}")

    # ---- Geometry constants (mm) ----------------------------------------
    JOINT_R = 30.0
    JOINT_H = 46.0
    LINK_W  = 20.0
    LINK_T  = 13.0

    BASE_COLOR  = (0.55, 0.55, 0.58)
    EE_COLOR    = (0.20, 0.72, 0.35)
    JNT_RIM     = tuple(c * 0.70 for c in joint_color)

    # ---- Frame chain ----------------------------------------------------
    DH, _, T_tcp, tcp_box = xarm6_params()
    T_frames = [np.eye(4)]
    for i in range(6):
        A = _sdh_transform(q[i] + DH[i,0], DH[i,1], DH[i,2], DH[i,3])
        T_frames.append(T_frames[-1] @ A)
    origins = np.array([T[:3, 3] for T in T_frames])   # (7, 3)

    # ---- Figure / axes --------------------------------------------------
    own_fig = (ax is None)
    if own_fig:
        fig = plt.figure(figsize=figsize)
        ax  = fig.add_subplot(111, projection='3d')
    else:
        fig = ax.get_figure()
        ax.cla()

    ax.set_facecolor('white')
    fig.patch.set_facecolor('white')
    _style_axes(ax)

    # ---- Axis limits ----------------------------------------------------
    if axes_lim is not None and len(axes_lim) == 6:
        xlim = (float(axes_lim[0]), float(axes_lim[1]))
        ylim = (float(axes_lim[2]), float(axes_lim[3]))
        zlim = (float(axes_lim[4]), float(axes_lim[5]))
    else:
        margin = 120.0
        xlim = (float(origins[:,0].min()-margin), float(origins[:,0].max()+margin))
        ylim = (float(origins[:,1].min()-margin), float(origins[:,1].max()+margin))
        zlim = (-60.0, float(origins[:,2].max()+margin))

    # ---- Links ----------------------------------------------------------
    for i in range(6):
        p1, p2 = origins[i], origins[i+1]
        if np.linalg.norm(p2 - p1) > 1e-3:
            _draw_beam(ax, p1, p2, LINK_W, LINK_T, link_color)

    # ---- Joint cylinders ------------------------------------------------
    for i in range(6):
        z_axis = T_frames[i][:3, 2]
        centre = origins[i]
        _draw_capped_cylinder(ax, centre - z_axis*JOINT_H, z_axis,
                              2*JOINT_H, JOINT_R, joint_color, JNT_RIM)

    # ---- Flange frame (shown only if show_flange=True) ------------------
    if show_flange:
        _draw_frame(ax, T_frames[6], frame_scale * 1.5, lw=2.5)

    # ---- TCP box + frame ------------------------------------------------
    T_tcp_world = T_frames[6] @ T_tcp
    R_tcp_w     = T_tcp_world[:3, :3]
    p_tcp       = T_tcp_world[:3, 3]
    ee_hw       = (tcp_box[1] - tcp_box[0]) / 2
    ee_ht       = (tcp_box[3] - tcp_box[2]) / 2
    z_tcp_w     = R_tcp_w[:, 2]
    p_box_start = p_tcp + tcp_box[4] * z_tcp_w
    p_box_end   = p_tcp + tcp_box[5] * z_tcp_w
    _draw_beam(ax, p_box_start, p_box_end, ee_hw, ee_ht, EE_COLOR)
    _draw_frame(ax, T_tcp_world, frame_scale * 1.5, lw=1.5)

    # ---- Base frame -----------------------------------------------------
    _draw_frame(ax, np.eye(4), frame_scale * 1.5, lw=2.5)

    # ---- Intermediate frames (optional) ---------------------------------
    if show_frames:
        for i in range(1, 6):
            _draw_frame(ax, T_frames[i], frame_scale, lw=1.5)

    # ---- Apply axis limits and force equal aspect ratio -----------------
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_zlim(*zlim)
    # Equal aspect ratio: cylinders render as circles, not ellipses.
    # Matplotlib 3.3+ supports set_aspect('equal') on 3D axes directly;
    # for older versions we fall back to manually scaling the box.
    try:
        ax.set_aspect('equal')
    except NotImplementedError:
        # Fallback: pad each axis to the same span
        spans = [xlim[1]-xlim[0], ylim[1]-ylim[0], zlim[1]-zlim[0]]
        max_span = max(spans)
        cx = (xlim[0]+xlim[1])/2;  ax.set_xlim(cx-max_span/2, cx+max_span/2)
        cy = (ylim[0]+ylim[1])/2;  ax.set_ylim(cy-max_span/2, cy+max_span/2)
        cz = (zlim[0]+zlim[1])/2;  ax.set_zlim(cz-max_span/2, cz+max_span/2)

    # ---- Title & view ---------------------------------------------------
    if title is None:
        q_str = ', '.join(f'{math.degrees(qi):.1f}' for qi in q)
        title = f'xArm 6   q = [{q_str}] deg'
    ax.set_title(title, fontsize=10, fontweight='normal')
    ax.view_init(elev=22, azim=45)

    return fig


# =========================================================================
#  Drawing primitives
# =========================================================================

def _style_axes(ax):
    ax.set_xlabel('X (mm)', fontsize=9)
    ax.set_ylabel('Y (mm)', fontsize=9)
    ax.set_zlabel('Z (mm)', fontsize=9)
    ax.tick_params(labelsize=7)
    for pane in (ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane):
        pane.fill = False
        pane.set_edgecolor((0.75, 0.75, 0.75))
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis._axinfo['grid']['color'] = (0.75, 0.75, 0.75, 0.8)


def _rot_z_to(v: np.ndarray) -> np.ndarray:
    """Rotation matrix mapping unit-z to direction v."""
    v  = v / np.linalg.norm(v)
    uz = np.array([0., 0., 1.])
    cr = np.cross(uz, v)
    sn = float(np.linalg.norm(cr))
    cs = float(np.dot(uz, v))
    if sn < 1e-9:
        return np.eye(3) * float(np.sign(cs))
    cr /= sn
    K = np.array([[0,-cr[2],cr[1]],[cr[2],0,-cr[0]],[-cr[1],cr[0],0]])
    return np.eye(3) + sn*K + (1-cs)*(K@K)


def _draw_capped_cylinder(ax, base, zdir, h, r, side_color, cap_color=None, n=32):
    if cap_color is None:
        cap_color = tuple(c*0.80 for c in side_color)
    R   = _rot_z_to(zdir)
    th  = np.linspace(0, 2*math.pi, n+1)
    ux, uy = np.cos(th), np.sin(th)
    top = base + h * (zdir / np.linalg.norm(zdir))
    ring_b = (R @ np.vstack([r*ux, r*uy, np.zeros(n+1)])).T + base
    ring_t = (R @ np.vstack([r*ux, r*uy, np.full(n+1, h)])).T + base
    # Barrel
    verts = [[ring_b[i], ring_b[i+1], ring_t[i+1], ring_t[i]] for i in range(n)]
    p = Poly3DCollection(verts, linewidth=0, alpha=0.92)
    p.set_facecolor(side_color); p.set_edgecolor('none')
    ax.add_collection3d(p)
    # Caps
    for ring, centre in [(ring_b, base), (ring_t, top)]:
        cap_verts = [[centre, ring[i], ring[i+1]] for i in range(n)]
        cp = Poly3DCollection(cap_verts, linewidth=0, alpha=0.92)
        cp.set_facecolor(cap_color); cp.set_edgecolor('none')
        ax.add_collection3d(cp)


def _draw_beam(ax, p1, p2, hw, ht, color):
    """Rectangular beam from p1 to p2, cross-section hw × ht."""
    vec = p2 - p1
    L   = float(np.linalg.norm(vec))
    if L < 1e-6:
        return
    d  = vec / L
    wz = np.array([0.,0.,1.])
    if abs(np.dot(d, wz)) > 0.99:
        wz = np.array([1.,0.,0.])
    y_ax = np.cross(d, wz); y_ax /= np.linalg.norm(y_ax)
    z_ax = np.cross(d, y_ax)
    R    = np.column_stack([d, y_ax, z_ax])
    c    = np.array([
        [0, hw, ht], [0,-hw, ht], [0,-hw,-ht], [0, hw,-ht],
        [L, hw, ht], [L,-hw, ht], [L,-hw,-ht], [L, hw,-ht],
    ])
    pts  = (R @ c.T).T + p1
    faces = [(0,1,2,3),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)]
    ec    = tuple(min(c*0.75, 1.0) for c in color)
    verts = [pts[list(f)] for f in faces]
    poly  = Poly3DCollection(verts, linewidth=0.5, alpha=0.92)
    poly.set_facecolor(color); poly.set_edgecolor(ec)
    ax.add_collection3d(poly)


def _draw_frame(ax, T, scale, lw):
    orig   = T[:3, 3]
    colors = ((0.85,0.15,0.15), (0.10,0.70,0.10), (0.15,0.40,0.85))
    labels = ('X', 'Y', 'Z')
    for i in range(3):
        tip = orig + scale * T[:3, i]
        ax.plot([orig[0],tip[0]], [orig[1],tip[1]], [orig[2],tip[2]],
                color=colors[i], linewidth=lw)
        ax.text(tip[0], tip[1], tip[2], labels[i],
                color=colors[i], fontsize=8, fontweight='bold')
