function [Q, num_solutions] = xarm6_ik(target, q_ref, varargin)
% XARM6_IK  Numerical inverse kinematics of the UFactory xArm 6
%
% Author : Lionel Birglen (with assistance from Anthropic Claude)
% Polytechnique Montreal, 2026
%
%   [Q, num_solutions] = xarm6_ik(target)
%   [Q, num_solutions] = xarm6_ik(target, q_ref)
%   [Q, num_solutions] = xarm6_ik(target, q_ref, 'Name', Value, ...)
%
%   Accepts the desired end-effector pose in any of the four orientation
%   conventions used by the UFactory SDK:
%
%   ┌─────────────────────────────────────────────────────────────────────┐
%   │  Convention          target argument                                │
%   ├─────────────────────────────────────────────────────────────────────┤
%   │  Homogeneous matrix  4×4 double T (base → flange)       [mm]       │
%   │  Position + RPY      struct with fields .p (3×1) [mm]              │
%   │                                         .rpy (3×1) [rad]           │
%   │                       rpy = [roll; pitch; yaw], XYZ fixed-angle    │
%   │                       R = Rz(yaw)*Ry(pitch)*Rx(roll)               │
%   │  Position + Quat     struct with fields .p (3×1) [mm]              │
%   │                                         .quat (4×1) [w;qx;qy;qz]  │
%   │  Position + AxAng    struct with fields .p (3×1) [mm]              │
%   │                                         .axis_angle (3×1) [rad]    │
%   │                       axis_angle = phi*[nx;ny;nz], phi >= 0        │
%   └─────────────────────────────────────────────────────────────────────┘
%
%   You may also pass the raw pose struct returned by xarm6_fk — all four
%   orientation fields are present, and .T is used directly.
%
%   Why a numerical approach:
%       The xArm 6 has a non-zero link offset on joint 5 (a5 = 76 mm),
%       meaning axes 4-5-6 do NOT intersect at a common point and the arm
%       does NOT have a true spherical wrist.
%
%   Inputs:
%       target   - Desired pose in any supported convention (see table above).
%       q_ref    - (optional) 6-element reference configuration [rad].
%                  Used as the first seed and to sort output by proximity.
%                  Defaults to zeros(1,6).
%
%   Optional name-value pairs:
%       'NSeeds'   - Number of random seeds (beyond q_ref and zeros).
%                    Default: 14.
%       'TolPos'   - Position convergence tolerance [mm].   Default: 0.05.
%       'TolRot'   - Rotation convergence tolerance [rad].  Default: 5e-4.
%       'MaxIter'  - Maximum Newton iterations per seed.    Default: 300.
%
%   Outputs:
%       Q            - Mx6 matrix of valid joint solutions [rad].
%                      Rows sorted by joint-space distance to q_ref.
%                      Empty (0×6) if no solution is found.
%       num_solutions- Number of valid solutions (= rows of Q).
%
%   Joint limits enforced:
%       q1 : [-2π,    +2π  ]   (±360°)
%       q2 : [-2.059, +2.059]  (±118°)
%       q3 : [-3.927, +0.192]  (-225° to +11°)
%       q4 : [-2π,    +2π  ]
%       q5 : [-1.692, +3.141]  (-97° to +180°)
%       q6 : [-2π,    +2π  ]
%	Unless something else is defined in xarm6_params.m
%
%   Examples:
%       %% 1. From a homogeneous matrix (original usage, unchanged)
%       q_true = deg2rad([30, -45, 0, 0, 45, -30]);
%       pose   = xarm6_fk(q_true);
%       [Q, n] = xarm6_ik(pose.T, q_true);
%
%       %% 2. From a pose struct (direct output of xarm6_fk)
%       [Q, n] = xarm6_ik(pose, q_true);
%
%       %% 3. From position + RPY
%       tgt.p   = [150; 80; 300];           % mm
%       tgt.rpy = deg2rad([10; -20; 45]);   % [roll; pitch; yaw] rad
%       [Q, n]  = xarm6_ik(tgt);
%
%       %% 4. From position + quaternion
%       tgt.p    = [150; 80; 300];
%       tgt.quat = [0.924; 0.0; 0.383; 0.0];  % [w; qx; qy; qz]
%       [Q, n]   = xarm6_ik(tgt);
%
%       %% 5. From position + UFactory axis-angle
%       tgt.p          = [150; 80; 300];
%       tgt.axis_angle = [1.57; 0; 0];   % phi*[nx;ny;nz]
%       [Q, n]         = xarm6_ik(tgt);
%
%   See also: xarm6_fk, xarm6_params

    % ---- Parse reference configuration -----------------------------------
    if nargin < 2 || isempty(q_ref)
        q_ref = zeros(1,6);
    end
    q_ref = q_ref(:)';

    % ---- Parse optional name-value pairs ---------------------------------
    pr = inputParser();
    addParameter(pr, 'NSeeds',  14,   @(x) isnumeric(x) && x >= 0);
    addParameter(pr, 'TolPos',  0.05, @isnumeric);
    addParameter(pr, 'TolRot',  5e-4, @isnumeric);
    addParameter(pr, 'MaxIter', 300,  @isnumeric);
    parse(pr, varargin{:});
    n_seeds  = pr.Results.NSeeds;
    tol_pos  = pr.Results.TolPos;
    tol_rot  = pr.Results.TolRot;
    max_iter = pr.Results.MaxIter;

    % ---- Convert target to 4×4 homogeneous matrix T_des ------------------
    T_des = parse_target(target);

    % ---- Joint limits (single source of truth: xarm6_params) ---------
    [~, qlim] = xarm6_params();
    lo = qlim(:,1)';
    hi = qlim(:,2)';

    % ---- Build seeds -----------------------------------------------------
    rng_state = rng(42, 'twister');
    seeds = [q_ref; zeros(1,6)];
    for k = 1:n_seeds
        seeds(end+1, :) = lo + rand(1,6) .* (hi - lo); 
    end
    rng(rng_state);

    % ---- Newton solver from each seed ------------------------------------
    Q_all = zeros(0,6);
    for k = 1:size(seeds,1)
        [q_sol, ok] = newton_ik(T_des, seeds(k,:), tol_pos, tol_rot, max_iter);
        if ~ok, continue; end

        pose_chk = xarm6_fk(q_sol);
        if norm(pose_chk.p - T_des(1:3,4)) > tol_pos * 10, continue; end

        if all(q_sol >= lo & q_sol <= hi)
            Q_all(end+1, :) = q_sol;
        end
    end

    % ---- Deduplicate & sort ----------------------------------------------
    if ~isempty(Q_all)
        Q_all = unique(round(Q_all, 3), 'rows');
        dists = sum((Q_all - q_ref).^2, 2);
        [~, idx] = sort(dists);
        Q_all = Q_all(idx, :);
    end

    Q             = Q_all;
    num_solutions = size(Q, 1);
end


% =========================================================================
%  Input parsing  –  any convention → 4×4 T
% =========================================================================

function T = parse_target(target)
% Convert any supported pose representation to a 4×4 homogeneous matrix.

    % --- Case 1: bare 4×4 matrix -----------------------------------------
    if isnumeric(target) && isequal(size(target), [4 4])
        T = double(target);
        return;
    end

    % --- Must be a struct from here on ------------------------------------
    if ~isstruct(target)
        error('xarm6_ik:badInput', ...
            ['target must be a 4×4 matrix or a struct with fields:\n' ...
             '  .p + .rpy        (position + roll-pitch-yaw)\n' ...
             '  .p + .quat       (position + quaternion [w;qx;qy;qz])\n' ...
             '  .p + .axis_angle (position + UFactory axis-angle)\n' ...
             '  .T               (full 4×4 homogeneous matrix)\n']);
    end

    % --- Case 2: struct that already has .T (e.g. output of xarm6_fk) ----
    if isfield(target, 'T')
        T = double(target.T);
        return;
    end

    % --- All remaining cases require a position vector -------------------
    if ~isfield(target, 'p')
        error('xarm6_ik:badInput', ...
            'Pose struct must contain a .p field (3-element position [mm]).');
    end
    p = double(target.p(:));
    if numel(p) ~= 3
        error('xarm6_ik:badInput', '.p must be a 3-element vector.');
    end

    % --- Case 3: position + RPY ------------------------------------------
    if isfield(target, 'rpy')
        rpy = double(target.rpy(:));
        if numel(rpy) ~= 3
            error('xarm6_ik:badInput', '.rpy must be a 3-element vector [roll;pitch;yaw].');
        end
        roll = rpy(1);  pitch = rpy(2);  yaw = rpy(3);
        % UFactory XYZ fixed-angle:  R = Rz(yaw)*Ry(pitch)*Rx(roll)
        R = Rz(yaw) * Ry(pitch) * Rx(roll);
        T = [R, p; 0 0 0 1];
        return;
    end

    % --- Case 4: position + quaternion -----------------------------------
    if isfield(target, 'quat')
        qv = double(target.quat(:));
        if numel(qv) ~= 4
            error('xarm6_ik:badInput', '.quat must be a 4-element vector [w;qx;qy;qz].');
        end
        R = quat2rot(qv);
        T = [R, p; 0 0 0 1];
        return;
    end

    % --- Case 5: position + UFactory axis-angle --------------------------
    if isfield(target, 'axis_angle')
        rv = double(target.axis_angle(:));
        if numel(rv) ~= 3
            error('xarm6_ik:badInput', '.axis_angle must be a 3-element vector [Rx;Ry;Rz].');
        end
        R = axisangle2rot(rv);
        T = [R, p; 0 0 0 1];
        return;
    end

    error('xarm6_ik:badInput', ...
        ['Pose struct must contain one of: .rpy, .quat, or .axis_angle\n' ...
         '(in addition to the required .p field).']);
end


% =========================================================================
%  Orientation conversion helpers
% =========================================================================

function R = Rx(a)
    R = [1, 0, 0; 0, cos(a), -sin(a); 0, sin(a), cos(a)];
end

function R = Ry(a)
    R = [cos(a), 0, sin(a); 0, 1, 0; -sin(a), 0, cos(a)];
end

function R = Rz(a)
    R = [cos(a), -sin(a), 0; sin(a), cos(a), 0; 0, 0, 1];
end

function R = quat2rot(q)
% Unit quaternion [w; qx; qy; qz] → 3×3 rotation matrix.
    q  = q / norm(q);   % normalise defensively
    w  = q(1);  qx = q(2);  qy = q(3);  qz = q(4);
    R  = [1-2*(qy^2+qz^2),  2*(qx*qy - w*qz),  2*(qx*qz + w*qy);
          2*(qx*qy + w*qz),  1-2*(qx^2+qz^2),  2*(qy*qz - w*qx);
          2*(qx*qz - w*qy),  2*(qy*qz + w*qx),  1-2*(qx^2+qy^2)];
end

function R = axisangle2rot(rv)
% UFactory axis-angle vector [Rx;Ry;Rz] = phi*[nx;ny;nz] → 3×3 rotation matrix.
% Uses Rodrigues' rotation formula.
    phi = norm(rv);
    if phi < 1e-10
        R = eye(3);
        return;
    end
    k = rv / phi;   % unit axis
    K = [0, -k(3),  k(2);
         k(3),  0, -k(1);
        -k(2),  k(1),  0];
    R = eye(3) + sin(phi)*K + (1 - cos(phi))*(K*K);
end


% =========================================================================
%  Newton IK – single seed
% =========================================================================

function [q, converged] = newton_ik(T_des, q0, tol_pos, tol_rot, max_iter)
% Damped-least-squares Newton solver.  Runs WITHOUT joint-limit clamping
% to avoid getting trapped at a boundary; limits are enforced by the caller.

    W_ROT    = 100.0;   % rotation weight: 1 rad equivalent to 100 mm
    LAMBDA   = 1e-4;    % damping constant
    MAX_STEP = 0.12;    % max joint delta per iteration [rad]

    q = q0(:)';

    for iter = 1:max_iter                          
        pose = xarm6_fk(q);
        Tc   = pose.T;

        dp = (T_des(1:3,4) - Tc(1:3,4))';          % 1×3  [mm]
        dR = T_des(1:3,1:3) * Tc(1:3,1:3)';
        dw = 0.5 * [dR(3,2)-dR(2,3), dR(1,3)-dR(3,1), dR(2,1)-dR(1,2)];  % 1×3 [rad]

        if norm(dp) < tol_pos && norm(dw) < tol_rot
            converged = true;
            return;
        end

        [Jp, Jr] = numerical_jacobian(q, Tc);

        err = [dp,  W_ROT * dw]';      % 6×1
        J   = [Jp;  W_ROT * Jr];       % 6×6

        dq = (J'*J + LAMBDA*eye(6)) \ (J'*err);
        dq = dq';   % 1×6

        scale = min(1.0, MAX_STEP / max(abs(dq)));
        q = q + scale * dq;
    end

    % Final convergence check
    pose = xarm6_fk(q);
    Tc   = pose.T;
    dp   = T_des(1:3,4) - Tc(1:3,4);
    dR   = T_des(1:3,1:3) * Tc(1:3,1:3)';
    dw   = 0.5 * [dR(3,2)-dR(2,3), dR(1,3)-dR(3,1), dR(2,1)-dR(1,2)];
    converged = norm(dp) < tol_pos && norm(dw) < tol_rot;
end


% =========================================================================
%  Numerical Jacobian
% =========================================================================

function [Jp, Jr] = numerical_jacobian(q, T0)
% Forward-difference numerical Jacobian.
% Returns position part Jp (3×6) and rotation part Jr (3×6).

    EPS = 1e-6;
    p0  = T0(1:3,4);
    Jp  = zeros(3,6);
    Jr  = zeros(3,6);

    for i = 1:6
        dq    = zeros(1,6);
        dq(i) = EPS;
        T1    = xarm6_fk(q + dq);
        T1    = T1.T;

        Jp(:,i) = (T1(1:3,4) - p0) / EPS;

        dR      = T1(1:3,1:3) * T0(1:3,1:3)';
        Jr(:,i) = [dR(3,2)-dR(2,3); dR(1,3)-dR(3,1); dR(2,1)-dR(1,2)] / (2*EPS);
    end
end
