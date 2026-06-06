function xarm6_plot(q, options)
% XARM6_PLOT  3-D visualisation of the UFactory xArm 6 robotic arm
%
% Author : Lionel Birglen (with assistance from Anthropic Claude)
% Polytechnique Montreal, 2026
%   xarm6_plot(q)
%   xarm6_plot(q, options)
%
%   Draws a 3-D model of the xArm 6 in the given joint configuration.
%   Each joint is rendered as a short cylinder whose axis coincides with
%   the joint's z-axis (axis of rotation in the DH convention).  Links are
%   drawn as rounded rectangular beams connecting consecutive joints.
%   A coordinate-frame triad is always shown at the end-effector, with
%   optional triads at every joint.
%
%   Inputs:
%       q        - 6-element vector of joint angles [rad]
%       options  - (optional) struct with any of these fields:
%                    .ax          - axes handle to plot into (new figure if omitted)
%                    .show_frames - draw intermediate frames (default: false)
%                    .show_flange - draw the flange frame triad (default: false)
%                    .link_color  - RGB for links   (default: [0.25 0.52 0.80])
%                    .joint_color - RGB for joints  (default: [0.90 0.35 0.20])
%                    .frame_scale - length of frame arrows [mm] (default: 60)
%                    .floor_length - floor rectangle length along X [mm] (default: 1200)
%                    .floor_width  - floor rectangle width  along Y [mm] (default: 900)
%                    .axes_lim     - 6-element vector [xmin xmax ymin ymax zmin zmax] [mm]
%                                    If absent, limits are set automatically from joint origins.
%                    .title_str   - custom figure title
%
%   Examples:
%       xarm6_plot(zeros(1,6));
%
%       opt.show_frames = true;
%       xarm6_plot(deg2rad([30 -45 0 0 45 -30]), opt);
%
%       % Animate
%       fig = figure; ax = axes(fig);
%       for t = linspace(0,2*pi,120)
%           q = [0.6*sin(t), -0.5+0.3*sin(2*t), 0, 0.4*sin(3*t), 0.5, 0.3*cos(t)];
%           q = max(min(q, [2*pi 2.059 0.192 2*pi 3.141 2*pi]), ...
%                      [-2*pi -2.059 -3.927 -2*pi -1.692 -2*pi]);
%           opt2.ax = ax; xarm6_plot(q,opt2); drawnow; pause(0.04);
%       end
%
%   See also: xarm6_fk, xarm6_ik, xarm6_params

    % ---- Parse options -------------------------------------------------
    if nargin < 1,  q=deg2rad([0;4.3;-24.4;-0.5;49.9;0.6]); end
    if nargin < 2,  options = struct(); end
    ax           = getopt(options, 'ax',           []);
    show_frames  = getopt(options, 'show_frames',  false);
    show_flange  = getopt(options, 'show_flange',  false);  % flange frame hidden by default
    link_color   = getopt(options, 'link_color',   [0.25 0.52 0.80]);
    joint_color  = getopt(options, 'joint_color',  [0.90 0.35 0.20]);
    frame_scale = getopt(options, 'frame_scale', 60);
    floor_length = getopt(options, 'floor_length', 1200);  % mm, along X
    floor_width  = getopt(options, 'floor_width',  1000);   % mm, along Y
    axes_lim     = getopt(options, 'axes_lim',     []);     % [xmin xmax ymin ymax zmin zmax]
    title_str   = getopt(options, 'title_str', ...
        sprintf('xArm 6   q = [%s] deg', ...
        strjoin(arrayfun(@(x)sprintf('%.1f',x), rad2deg(q), ...
        'UniformOutput',false), ', ')));

    % ---- Geometry constants (mm) ---------------------------------------
    JOINT_R  = 30;     % joint cylinder radius
    JOINT_H  = 46;     % joint cylinder half-height (total height = 2*JOINT_H)
    LINK_W   = 20;     % link beam half-width
    LINK_T   = 13;     % link beam half-thickness
    BASE_R   = 82;     % base disc radius
    BASE_H   = 52;     % base disc height

    % Colours
    BASE_COLOR = [0.55 0.55 0.58];
    LINK_COLOR = link_color;
    JNT_COLOR  = joint_color;
    JNT_RIM    = joint_color * 0.70;  % darker ring caps
    EE_COLOR   = [0.20 0.72 0.35];   % green — end-effector / tool beam

    % ---- Compute all frame poses (0 = world/base, 1..6 = after joint i) -
    [DH, ~, T_tcp, tcp_box] = xarm6_params();

    T_frames = zeros(4,4,7);
    T_frames(:,:,1) = eye(4);
    for i = 1:6
        T_frames(:,:,i+1) = T_frames(:,:,i) * ...
            sdh_T(q(i)+DH(i,1), DH(i,2), DH(i,3), DH(i,4));
    end

    origins = squeeze(T_frames(1:3,4,:));   % 3×7

    % ---- Figure / axes ------------------------------------------------
    if isempty(ax)
        fig = figure('Name','xArm 6 Visualiser', 'Color','white', ...
                     'NumberTitle','off');
        ax  = axes(fig);
        newFig = true;
    else
        cla(ax);
        newFig = false;
    end

    hold(ax,'on');
    set(ax, 'DataAspectRatio',[1 1 1], ...
            'Color','white', ...
            'XColor',[0.30 0.30 0.30], ...
            'YColor',[0.30 0.30 0.30], ...
            'ZColor',[0.30 0.30 0.30], ...
            'GridColor',[0.75 0.75 0.75], ...
            'GridAlpha', 0.8, ...
            'LineWidth', 0.8);
    grid(ax,'on');
    box(ax,'on');
    xlabel(ax,'X (mm)', 'FontSize',9);
    ylabel(ax,'Y (mm)', 'FontSize',9);
    zlabel(ax,'Z (mm)', 'FontSize',9);
    title(ax, title_str, 'FontSize',10, 'FontWeight','normal');
    if newFig
        set(fig,'Color','white');
    end

    % ---- Base Floor -----------------------------------------------------
    % Grey rectangle at z = 0, centred on the robot base (x=0, y=0).
    % Dimensions controlled by options.floor_length (X) and options.floor_width (Y).
    FLOOR_COLOR = [0.82 0.82 0.82];
    hl = floor_length / 2;   % half-length along X
    hw = floor_width  / 2;   % half-width  along Y
    fill3(ax, [-hl  hl  hl -hl], [-hw -hw  hw  hw], [0 0 0 0], ...
          FLOOR_COLOR, 'EdgeColor', [0.60 0.60 0.60], 'LineWidth', 0.8, ...
          'AmbientStrength', 0.6, 'DiffuseStrength', 0.4, 'FaceAlpha', 0.8);

    % ---- Links (beams between joint origins) ---------------------------
    for i = 1:6
        p1  = origins(:,i);
        p2  = origins(:,i+1);
        if norm(p2-p1) > 1e-3
            draw_beam(ax, p1, p2, LINK_W, LINK_T, LINK_COLOR);
        end
    end

    % ---- Joint cylinders (axis = z-axis of frame BEFORE the joint) -----
    %   Frame i (T_frames(:,:,i)) has its z-axis = column 3.
    %   Joint i rotates about z of frame i, centred at origins(:,i+1).
    for i = 1:6
        z_axis = T_frames(1:3,3,i);       % z of the parent frame
        centre = origins(:,i);           % joint i origin
        draw_capped_cylinder(ax, centre - z_axis*JOINT_H, z_axis, ...
                             2*JOINT_H, JOINT_R, JNT_COLOR, JNT_RIM);
    end

    % ---- End-effector (flange) frame triad (shown only if show_flange=true) -
    if show_flange
        draw_frame(ax, T_frames(:,:,7), frame_scale*1.5, 2.5);
    end

    % ---- TCP box and TCP frame triad ------------------------------------
    % The tool bounding box is drawn centred on the TCP, with its axes
    % aligned to the TCP frame.  The beam runs along the TCP z-axis from
    % tcp_box(5) to tcp_box(6), with cross-section given by the x and y extents.
    T_tcp_world = T_frames(:,:,7) * T_tcp;
    R_tcp_w     = T_tcp_world(1:3,1:3);
    p_tcp       = T_tcp_world(1:3,4);
    ee_hw       = (tcp_box(2) - tcp_box(1)) / 2;   % half-width  (x extent)
    ee_ht       = (tcp_box(4) - tcp_box(3)) / 2;   % half-thickness (y extent)
    z_tcp_w     = R_tcp_w(:,3);                     % TCP z-axis in world frame
    % Endpoints of the beam along TCP z, offset from TCP origin
    p_box_start = p_tcp + tcp_box(5) * z_tcp_w;
    p_box_end   = p_tcp + tcp_box(6) * z_tcp_w;
    draw_beam(ax, p_box_start, p_box_end, ee_hw, ee_ht, EE_COLOR);
    draw_frame(ax, T_tcp_world, frame_scale*1.5, 1.5);

    % ---- Base frame triad (always shown) -----------------------
    draw_frame(ax, eye(4), frame_scale*1.5, 2.5);

    % ---- Intermediate frames (optional) --------------------------------
    if show_frames
        for i = 2:6
            draw_frame(ax, T_frames(:,:,i), frame_scale, 1.5);
        end
    end

    % ---- Axis limits ---------------------------------------------------
    if numel(axes_lim) == 6
        % User-supplied limits: [xmin xmax ymin ymax zmin zmax]
        xlim(ax, axes_lim(1:2));
        ylim(ax, axes_lim(3:4));
        zlim(ax, axes_lim(5:6));
    else
        % Automatic limits derived from joint origins
        margin = 120;
        xlim(ax, [min(origins(1,:))-margin,  max(origins(1,:))+margin]);
        ylim(ax, [min(origins(2,:))-margin,  max(origins(2,:))+margin]);
        zlim(ax, [-60,                        max(origins(3,:))+margin]);
    end

    % ---- Lighting & view ----------------------------------------------
    view(ax, 45, 22);
    camlight(ax, 'headlight');
    lighting(ax, 'gouraud');
    material(ax, 'dull');

    hold(ax,'off');
end


% =========================================================================
%  Drawing primitives
% =========================================================================

function draw_capped_cylinder(ax, base, zdir, h, r, side_color, cap_color)
% Solid cylinder from BASE along ZDIR of height H and radius R,
% with flat caps. side_color for the barrel; cap_color (optional) for caps.
    if nargin < 7, cap_color = side_color * 0.80; end
    N  = 32;
    th = linspace(0, 2*pi, N+1);
    ux = cos(th);  uy = sin(th);
    R  = rot_z_to(zdir);
    top = base + h * (zdir / norm(zdir));
    ring_b = R * [r*ux; r*uy; zeros(1,N+1)];
    ring_t = R * [r*ux; r*uy; ones(1,N+1)*h];
    Xb = ring_b(1,:) + base(1);  Yb = ring_b(2,:) + base(2);  Zb = ring_b(3,:) + base(3);
    Xt = ring_t(1,:) + base(1);  Yt = ring_t(2,:) + base(2);  Zt = ring_t(3,:) + base(3);
    % Barrel
    surf(ax, [Xb;Xt], [Yb;Yt], [Zb;Zt], 'FaceColor', side_color, 'EdgeColor', 'none', ...
         'AmbientStrength',0.35, 'DiffuseStrength',0.75, ...
         'SpecularStrength',0.30, 'SpecularExponent',20);
    % Caps
    draw_disc(ax, base, Xb, Yb, Zb, cap_color);
    draw_disc(ax, top,  Xt, Yt, Zt, cap_color);
end

function draw_disc(ax, centre, Xr, Yr, Zr, color)
% Filled disc: triangles from centre to each ring edge segment.
    N = numel(Xr) - 1;
    X = [repmat(centre(1),1,N); Xr(1:N); Xr(2:N+1)];
    Y = [repmat(centre(2),1,N); Yr(1:N); Yr(2:N+1)];
    Z = [repmat(centre(3),1,N); Zr(1:N); Zr(2:N+1)];
    fill3(ax, X, Y, Z, color, 'EdgeColor','none', ...
          'AmbientStrength',0.35, 'DiffuseStrength',0.65);
end

function draw_beam(ax, p1, p2, hw, ht, color)
% Draw a rectangular beam from p1 to p2 with half-width hw and half-thickness ht.
% The cross-section is oriented so the wide face is perpendicular to gravity (z).
    vec = p2 - p1;
    L   = norm(vec);
    if L < 1e-6, return; end
    d   = vec / L;

    % Build a local frame: x=along beam, y=wide, z=thin
    % Choose y perpendicular to d and to world-z where possible
    wz = [0;0;1];
    if abs(dot(d, wz)) > 0.99
        wz = [1;0;0];
    end
    y_ax = cross(d, wz);  y_ax = y_ax / norm(y_ax);
    z_ax = cross(d, y_ax);

    % 8 corners of the beam box
    corners_local = [ ...
        0,  hw,  ht;   0, -hw,  ht; ...
        0, -hw, -ht;   0,  hw, -ht; ...
        L,  hw,  ht;   L, -hw,  ht; ...
        L, -hw, -ht;   L,  hw, -ht ];

    % Transform to world frame
    R   = [d, y_ax, z_ax];
    pts = (R * corners_local')' + p1';    % 8×3

    % Draw 6 faces
    faces = {[1 2 3 4], [5 6 7 8], [1 2 6 5], [2 3 7 6], [3 4 8 7], [4 1 5 8]};
    for f = 1:6
        idx = faces{f};
        fill3(ax, pts(idx,1), pts(idx,2), pts(idx,3), color, ...
              'EdgeColor', color*0.75, 'LineWidth', 0.5, ...
              'AmbientStrength',0.35, 'DiffuseStrength',0.75, ...
              'SpecularStrength',0.20, 'SpecularExponent',15);
    end
end

function draw_frame(ax, T, scale, lw, dashed)
% Draw an XYZ triad at frame T.
% dashed (optional, default false): use '--' line style (for TCP frame).
    if nargin < 5, dashed = false; end
    if dashed, ls = '--'; else, ls = '-'; end
    orig = T(1:3,4);
    cols = {[0.85 0.15 0.15], [0.10 0.70 0.10], [0.15 0.40 0.85]};
    lbls = {'X','Y','Z'};
    for k = 1:3
        tip = orig + scale * T(1:3,k);
        plot3(ax, [orig(1) tip(1)], [orig(2) tip(2)], [orig(3) tip(3)], ...
              ls, 'Color', cols{k}, 'LineWidth', lw);
        text(ax, tip(1), tip(2), tip(3), lbls{k}, ...
             'Color', cols{k}, 'FontSize', 8, 'FontWeight', 'bold');
    end
end


% =========================================================================
%  Geometry helpers
% =========================================================================

function R = rot_z_to(v)
% Rotation matrix that maps unit-z to direction v.
    v  = v / norm(v);
    uz = [0;0;1];
    cr = cross(uz, v);
    sn = norm(cr);
    cs = dot(uz, v);
    if sn < 1e-9
        R = eye(3) * sign(cs);
        return;
    end
    cr = cr / sn;
    K  = [0,-cr(3),cr(2); cr(3),0,-cr(1); -cr(2),cr(1),0];
    R  = eye(3) + sn*K + (1-cs)*(K*K);
end

function A = sdh_T(theta, d, a, alpha)
    ct = cos(theta); st = sin(theta);
    ca = cos(alpha); sa = sin(alpha);
    A  = [ ct, -st*ca,  st*sa,  a*ct ;
           st,  ct*ca, -ct*sa,  a*st ;
            0,     sa,     ca,     d ;
            0,      0,      0,     1 ];
end

function v = getopt(s, field, default)
    if isfield(s, field), v = s.(field); else, v = default; end
end
