function [DH, qlim, T_tcp, tcp_box] = xarm6_params()
% XARM6_PARAMS  Returns the Standard DH parameter table, joint limits,
%               TCP offset, and TCP bounding box for the UFactory xArm 6.
%
% Author : Lionel Birglen (with assistance from Anthropic Claude)
% Polytechnique Montreal, 2026
%
%   [DH, qlim, T_tcp, tcp_box] = xarm6_params()
%
%   Outputs:
%       DH      - 6x4 matrix, each row = [theta_offset, d (mm), a (mm), alpha (rad)]
%                 The actual theta for joint i is:  theta_i = q(i) + DH(i,1)
%
%       qlim    - 6x2 matrix of joint limits [lower, upper] in radians,
%                 each row corresponding to one joint.
%
%       T_tcp   - 4x4 homogeneous transformation from the flange (frame 6)
%                 to the Tool Centre Point (TCP).  Edit TCP_translation and
%                 TCP_rotation below to match your physical tool.
%                 Default: identity (TCP coincides with the flange).
%
%       tcp_box - 6-element vector defining the tool bounding box in the
%                 local TCP frame [mm]:
%                   [x_min, x_max, y_min, y_max, z_min, z_max]
%                 The box is centred on the TCP origin and its faces are
%                 aligned with the TCP axes.  xarm6_plot uses this box to
%                 draw the end-effector beam with the correct shape.
%                 Edit TCP_box below to match your physical tool geometry.
%                 Default: a small 20x20x20 mm cube centred at the TCP.
%
%   Standard D-H parameters (UFactory official, all lengths in mm):
%   ---------------------------------------------------------------
%   Joint | theta_offset | d [mm] | a [mm]      | alpha [rad]
%   ------+--------------+--------+-------------+-------------
%     1   |     0        | 267.0  |    0         | -pi/2
%     2   |   T2_off     |   0    |  a2 (289.49) |  0
%     3   |   T3_off     |   0    |  77.5        | -pi/2
%     4   |     0        | 342.5  |    0         | +pi/2
%     5   |     0        |   0    |  76.0        | -pi/2
%     6   |     0        |  97.0  |    0         |  0
%   ---------------------------------------------------------------
%
%   Derived constants:
%       a2      = sqrt(284.5^2 + 53.5^2) = 289.48866 mm
%       T2_off  = -atan2(284.5, 53.5)    = -1.3849179 rad (-79.34995 deg)
%       T3_off  = +atan2(284.5, 53.5)    = +1.3849179 rad (+79.34995 deg)
%
%   Joint limits [lower, upper] in radians:
%   ---------------------------------------------------------------
%   Joint | Lower          | Upper          | Range
%   ------+----------------+----------------+----------------------
%     1   | -2*pi          | +2*pi          | ±360 deg
%     2   | -2.059         | +2.059         | ±118 deg
%     3   | -3.927         | +0.192         | -225 deg to +11 deg
%     4   | -2*pi          | +2*pi          | ±360 deg
%     5   | -1.692         | +3.141         | -97 deg to +180 deg
%     6   | -2*pi          | +2*pi          | ±360 deg
%   ---------------------------------------------------------------

    a2     = sqrt(284.5^2 + 53.5^2);
    T2_off = -atan2(284.5, 53.5);
    T3_off =  -T2_off;

    %         theta_off   d       a      alpha
    DH = [    0,        267.0,   0,    -pi/2 ;
              T2_off,     0,    a2,     0    ;
              T3_off,     0,   77.5,  -pi/2 ;
              0,        342.5,   0,    pi/2  ;
              0,          0,   76.0,  -pi/2 ;
              0,         97.0,   0,    0    ];

    %          lower     upper
    qlim = [ -2*pi,    2*pi   ;   % q1  ±360 deg
             -2.059,   2.059  ;   % q2  ±118 deg
             -3.927,   0.192  ;   % q3  -225 deg to +11 deg
             -2*pi,    2*pi   ;   % q4  ±360 deg
             -1.692,   3.141  ;   % q5  -97 deg to +180 deg
             -2*pi,    2*pi   ];  % q6  ±360 deg

    % ---- TCP offset (flange → TCP) --------------------------------------
    % Edit the values below to match your physical tool.
    % T_tcp is a 4x4 homogeneous transform:
    %   - Upper-left 3x3: rotation of the TCP frame relative to the flange
    %   - Right column 1:3: translation of the TCP origin in flange coords [mm]
    % Default: identity — TCP coincides with the flange frame.
    TCP_translation = [0; 0; 50];    % [mm]  e.g. [0; 0; 100] for 100 mm along flange z
    TCP_rotation    = eye(3);       % no rotation relative to flange
    T_tcp = [TCP_rotation, TCP_translation; 0 0 0 1];

    % ---- Tool bounding box in local TCP frame [mm] ----------------------
    % Defines the extents of the physical tool for visualisation.
    % Format: [x_min, x_max, y_min, y_max, z_min, z_max]
    % The beam drawn in xarm6_plot spans from the flange to the TCP and is
    % shaped using the cross-section (x and y extents) of this box.
    % Edit these values to match your tool dimensions.
    tcp_box = [-5, 5, -10, 10, -50, 0];   % mm — box hangs below TCP origin
end
