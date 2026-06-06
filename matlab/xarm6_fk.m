function pose = xarm6_fk(q)
% XARM6_FK  Forward kinematics of the UFactory xArm 6
%
% Author : Lionel Birglen (with assistance from Anthropic Claude)
% Polytechnique Montreal, 2026
%
%   pose = xarm6_fk(q)
%
%   Computes the end-effector pose from joint angles and returns it in
%   every orientation convention used by the UFactory SDK.
%
%   Input:
%       q    - 6-element vector of joint angles [rad]
%
%   Output:
%       pose - struct with the following fields:
%
%         Flange frame (base → flange, i.e. DH frame 6):
%         .T          (4x4)  Homogeneous transformation matrix  [mm]
%         .p          (3x1)  Flange position [mm]:  [x; y; z]
%         .rpy        (3x1)  Roll-Pitch-Yaw [rad] — XYZ fixed-angle (UFactory)
%                              R = Rz(yaw) * Ry(pitch) * Rx(roll)
%                              Order: [roll; pitch; yaw]
%         .quat       (4x1)  Unit quaternion [w; qx; qy; qz], w >= 0
%         .axis_angle (3x1)  UFactory Rx/Ry/Rz vector [rad]:
%                              [Rx;Ry;Rz] = phi*[nx;ny;nz], phi >= 0
%
%         TCP frame (base → TCP = flange * T_tcp from xarm6_params):
%         .T_tcp          (4x4)  Homogeneous transformation matrix  [mm]
%         .p_tcp          (3x1)  TCP position [mm]:  [x; y; z]
%         .rpy_tcp        (3x1)  Roll-Pitch-Yaw of TCP [rad]
%         .quat_tcp       (4x1)  Unit quaternion of TCP [w; qx; qy; qz]
%         .axis_angle_tcp (3x1)  UFactory axis-angle of TCP [rad]
%
%         To change the TCP offset, edit TCP_translation and TCP_rotation
%         in xarm6_params.m.
%
%   Orientation conventions reference:
%       UFactory xArm / 850 / Lite 6 User Manual – "Terms and Definitions"
%       RPY  : XYZ fixed-angle  (R = Rz*Ry*Rx)
%       Rx/Ry/Rz : axis-angle compact vector, phi >= 0
%       Quaternion derived from axis-angle: q = [cos(phi/2); sin(phi/2)*axis]
%
%   DH parameters (Standard convention, all lengths in mm):
%   ---------------------------------------------------------------
%   Joint | theta_offset  | d [mm]  | a [mm]      | alpha
%   ------+---------------+---------+-------------+----------
%     1   | 0             | 267.0   | 0            | -pi/2
%     2   | T2_off        | 0       | a2 ~289.49   |  0
%     3   | T3_off        | 0       | 77.5         | -pi/2
%     4   | 0             | 342.5   | 0            | +pi/2
%     5   | 0             | 0       | 76.0         | -pi/2
%     6   | 0             | 97.0    | 0            |  0
%   ---------------------------------------------------------------
%   a2     = sqrt(284.5^2 + 53.5^2)  ~= 289.4887 mm
%   T2_off = -atan2(284.5, 53.5)     ~= -1.3849 rad
%   T3_off = -T2_off
%
%   Example:
%       q    = deg2rad([30, -45, 0, 0, 45, -30]);
%       pose = xarm6_fk(q);
%       fprintf('Position  : [%.2f, %.2f, %.2f] mm\n',  pose.p);
%       fprintf('RPY       : [%.4f, %.4f, %.4f] rad\n', pose.rpy);
%       fprintf('Quaternion: [%.4f, %.4f, %.4f, %.4f]\n', pose.quat);
%       fprintf('Axis-angle: [%.4f, %.4f, %.4f] rad\n', pose.axis_angle);
%
%   See also: xarm6_ik, xarm6_params

    q = q(:)';   % ensure row vector

    % ---- DH parameters and TCP offset (single source of truth) ----------
    [DH, ~, T_tcp] = xarm6_params();

    % ---- Build flange transform T (base → flange) -----------------------
    T = eye(4);
    for i = 1:6
        theta = q(i) + DH(i,1);
        d     = DH(i,2);
        a     = DH(i,3);
        alpha = DH(i,4);
        T = T * sdh_transform(theta, d, a, alpha);
    end

    % ---- TCP transform (base → TCP) -------------------------------------
    T_tcp_world = T * T_tcp;
    R_tcp = T_tcp_world(1:3,1:3);
    p_tcp = T_tcp_world(1:3,4);

    R = T(1:3,1:3);
    p = T(1:3,4);

    % ---- RPY  (XYZ fixed-angle, UFactory convention) — flange ----------
    pitch = atan2(-R(3,1), sqrt(R(1,1)^2 + R(2,1)^2));
    if abs(cos(pitch)) > 1e-10
        roll = atan2( R(3,2), R(3,3));
        yaw  = atan2( R(2,1), R(1,1));
    else
        roll = 0;
        if pitch > 0
            yaw = atan2( R(1,2), R(2,2));
        else
            yaw = atan2(-R(1,2), R(2,2));
        end
    end
    rpy = [roll; pitch; yaw];

    % ---- Quaternion and axis-angle — flange ----------------------------
    quat       = rot2quat(R);
    axis_angle = quat2axisangle(quat);

    % ---- Same orientation representations for TCP ----------------------
    pitch_tcp = atan2(-R_tcp(3,1), sqrt(R_tcp(1,1)^2 + R_tcp(2,1)^2));
    if abs(cos(pitch_tcp)) > 1e-10
        roll_tcp = atan2( R_tcp(3,2), R_tcp(3,3));
        yaw_tcp  = atan2( R_tcp(2,1), R_tcp(1,1));
    else
        roll_tcp = 0;
        if pitch_tcp > 0
            yaw_tcp = atan2( R_tcp(1,2), R_tcp(2,2));
        else
            yaw_tcp = atan2(-R_tcp(1,2), R_tcp(2,2));
        end
    end
    quat_tcp       = rot2quat(R_tcp);
    axis_angle_tcp = quat2axisangle(quat_tcp);

    % ---- Assemble output struct ----------------------------------------
    % Flange fields
    pose.T          = T;
    pose.p          = p;
    pose.rpy        = rpy;
    pose.quat       = quat;
    pose.axis_angle = axis_angle;
    % TCP fields
    pose.T_tcp      = T_tcp_world;
    pose.p_tcp      = p_tcp;
    pose.rpy_tcp        = [roll_tcp; pitch_tcp; yaw_tcp];
    pose.quat_tcp       = quat_tcp;
    pose.axis_angle_tcp = axis_angle_tcp;
end


% =========================================================================
%  Private helper functions
% =========================================================================

function A = sdh_transform(theta, d, a, alpha)
% Standard DH link transform: Rz(theta)*Tz(d)*Tx(a)*Rx(alpha)
    ct = cos(theta);  st = sin(theta);
    ca = cos(alpha);  sa = sin(alpha);
    A  = [ ct, -st*ca,  st*sa,  a*ct ;
           st,  ct*ca, -ct*sa,  a*st ;
            0,     sa,     ca,     d ;
            0,      0,      0,     1 ];
end


function q = rot2quat(R)
% Convert 3x3 rotation matrix to unit quaternion [w; qx; qy; qz].
% Uses Shepperd's method to avoid division by near-zero values.
    trace_R = R(1,1) + R(2,2) + R(3,3);

    % Pick the largest of {w, qx, qy, qz} as the pivot for numerical stability
    candidates = [trace_R, R(1,1), R(2,2), R(3,3)];
    [~, idx] = max(candidates);

    switch idx
        case 1   % w is largest
            w  =  0.5 * sqrt(1 + trace_R);
            s  =  0.25 / w;
            qx = (R(3,2) - R(2,3)) * s;
            qy = (R(1,3) - R(3,1)) * s;
            qz = (R(2,1) - R(1,2)) * s;
        case 2   % qx is largest
            qx =  0.5 * sqrt(1 + R(1,1) - R(2,2) - R(3,3));
            s  =  0.25 / qx;
            w  = (R(3,2) - R(2,3)) * s;
            qy = (R(1,2) + R(2,1)) * s;
            qz = (R(1,3) + R(3,1)) * s;
        case 3   % qy is largest
            qy =  0.5 * sqrt(1 - R(1,1) + R(2,2) - R(3,3));
            s  =  0.25 / qy;
            w  = (R(1,3) - R(3,1)) * s;
            qx = (R(1,2) + R(2,1)) * s;
            qz = (R(2,3) + R(3,2)) * s;
        case 4   % qz is largest
            qz =  0.5 * sqrt(1 - R(1,1) - R(2,2) + R(3,3));
            s  =  0.25 / qz;
            w  = (R(2,1) - R(1,2)) * s;
            qx = (R(1,3) + R(3,1)) * s;
            qy = (R(2,3) + R(3,2)) * s;
    end

    q = [w; qx; qy; qz];
    q = q / norm(q);   % normalise to unit length

    % Canonical form: ensure w >= 0  (negate whole quaternion if not)
    if q(1) < 0
        q = -q;
    end
end


function rv = quat2axisangle(q)
% Convert unit quaternion [w; qx; qy; qz] to UFactory axis-angle vector
%   [Rx; Ry; Rz] = phi * [nx; ny; nz],   phi >= 0
    w  = q(1);
    qv = q(2:4);

    % Clamp w to [-1,1] before acos to guard against floating-point overshoot
    phi = 2 * acos(max(-1, min(1, w)));   % in [0, 2*pi]

    % Keep phi in [0, pi]: if phi > pi, negate axis and use 2*pi - phi
    if phi > pi
        phi = 2*pi - phi;
        qv  = -qv;
    end

    sn = norm(qv);
    if sn < 1e-10
        % Near-identity rotation: axis undefined, return zero vector
        rv = [0; 0; 0];
    else
        rv = (phi / sn) * qv;   % [Rx; Ry; Rz] = phi * unit_axis
    end
end
