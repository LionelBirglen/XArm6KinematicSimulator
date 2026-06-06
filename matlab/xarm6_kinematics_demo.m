%% xarm6_kinematics_demo.m
% Demonstration and validation of xarm6_fk and xarm6_ik.
%
% Author : Lionel Birglen (with assistance from Anthropic Claude)
% Polytechnique Montreal, 2026
%
% Sections:
%   1. FK – display all orientation representations
%   2. Round-trip: verify each representation reconstructs the rotation matrix
%   3. IK – all five input conventions (T, pose struct, RPY, quat, axis-angle)
%   4. Monte-Carlo accuracy test (100 random configurations)
%
% Requires: xarm6_fk.m, xarm6_ik.m, xarm6_params.m
% -------------------------------------------------------------------------
clear; clc;
fprintf('=== xArm6 Kinematics Demo ===\n\n');

% ---- Shared test configuration (within joint limits) --------------------
q_true = deg2rad([30, -45, 0, 0, 45, -30]);
pose   = xarm6_fk(q_true);

% =========================================================================
%% 1. Forward kinematics – all orientation representations
% =========================================================================
fprintf('--- FK at q = [30, -45, 0, 0, 45, -30] deg ---\n');
fprintf('Position    (mm)  : x=%.3f  y=%.3f  z=%.3f\n', pose.p);
fprintf('RPY         (rad) : roll=%.6f  pitch=%.6f  yaw=%.6f\n',  pose.rpy);
fprintf('RPY         (deg) : roll=%.4f  pitch=%.4f  yaw=%.4f\n',  rad2deg(pose.rpy));
fprintf('Quaternion        : w=%.6f  qx=%.6f  qy=%.6f  qz=%.6f\n', pose.quat);
fprintf('Axis-angle  (rad) : Rx=%.6f  Ry=%.6f  Rz=%.6f\n', pose.axis_angle);
phi = norm(pose.axis_angle);
if phi > 1e-10
    ax = pose.axis_angle / phi;
    fprintf('  phi = %.6f rad (%.4f deg),  axis = [%.4f, %.4f, %.4f]\n', ...
            phi, rad2deg(phi), ax);
end
fprintf('\n');

% =========================================================================
%% 2. Round-trip validation: each representation -> rotation matrix
% =========================================================================
fprintf('--- Orientation round-trip errors (Frobenius norm vs R from T) ---\n');
R_ref = pose.T(1:3,1:3);

% RPY -> R
roll = pose.rpy(1);  pitch = pose.rpy(2);  yaw = pose.rpy(3);
R_rpy = Rz(yaw) * Ry(pitch) * Rx(roll);
fprintf('RPY        -> R  error: %.2e\n', norm(R_ref - R_rpy, 'fro'));

% Quaternion -> R
qv = pose.quat;
w = qv(1); qx = qv(2); qy = qv(3); qz = qv(4);
R_quat = [1-2*(qy^2+qz^2), 2*(qx*qy-w*qz),  2*(qx*qz+w*qy);
          2*(qx*qy+w*qz),  1-2*(qx^2+qz^2),  2*(qy*qz-w*qx);
          2*(qx*qz-w*qy),  2*(qy*qz+w*qx),  1-2*(qx^2+qy^2)];
fprintf('Quaternion -> R  error: %.2e\n', norm(R_ref - R_quat, 'fro'));

% Axis-angle -> R  (Rodrigues)
rv  = pose.axis_angle;
phi = norm(rv);
if phi > 1e-10
    k   = rv / phi;
    K   = [0,-k(3),k(2); k(3),0,-k(1); -k(2),k(1),0];
    R_aa = eye(3) + sin(phi)*K + (1-cos(phi))*(K*K);
else
    R_aa = eye(3);
end
fprintf('Axis-angle -> R  error: %.2e\n\n', norm(R_ref - R_aa, 'fro'));

% =========================================================================
%% 3. Inverse kinematics – all five input conventions
% =========================================================================
fprintf('=== IK from all five input conventions ===\n\n');

% 3a. 4x4 homogeneous matrix
[Q1, n1] = xarm6_ik(pose.T, q_true);
report_ik('4x4 matrix', Q1, n1, pose);

% 3b. Full pose struct (direct output of xarm6_fk)
[Q2, n2] = xarm6_ik(pose, q_true);
report_ik('pose struct (.T)', Q2, n2, pose);

% 3c. Position + RPY
tgt_rpy.p   = pose.p;
tgt_rpy.rpy = pose.rpy;
[Q3, n3] = xarm6_ik(tgt_rpy, q_true);
report_ik('position + RPY', Q3, n3, pose);

% 3d. Position + quaternion
tgt_quat.p    = pose.p;
tgt_quat.quat = pose.quat;
[Q4, n4] = xarm6_ik(tgt_quat, q_true);
report_ik('position + quaternion', Q4, n4, pose);

% 3e. Position + UFactory axis-angle
tgt_aa.p          = pose.p;
tgt_aa.axis_angle = pose.axis_angle;
[Q5, n5] = xarm6_ik(tgt_aa, q_true);
report_ik('position + axis-angle', Q5, n5, pose);

% Check all conventions agree on the best solution
if n1>0 && n3>0 && n4>0 && n5>0
    fprintf('All-convention agreement (max diff in best solution): %.2e rad\n\n', ...
        max([norm(Q1(1,:)-Q3(1,:)), norm(Q1(1,:)-Q4(1,:)), norm(Q1(1,:)-Q5(1,:))]));
end

% =========================================================================
%% 4. Monte-Carlo round-trip test
% =========================================================================
fprintf('--- Monte-Carlo round-trip test (100 random configs) ---\n');

[~, QLIM] = xarm6_params();   % joint limits from single source of truth

N_test   = 100;
pos_errs = nan(N_test,1);
rot_errs = nan(N_test,1);
no_sol   = 0;

BAR_WIDTH  = 40;   % number of '#' characters at 100%
% Each redraw erases the previous bar with backspaces.
% Bar format: "  [####----] 100%"  = 2 + 1 + BAR_WIDTH + 1 + 1 + 4 = BAR_WIDTH+9 chars
ERASE = repmat('\b', 1, BAR_WIDTH + 9);

fprintf('  [%s]   0%%', repmat('-', 1, BAR_WIDTH));   % initial empty bar

rng(42);
for t = 1:N_test
    % Erase previous bar then redraw on the same line
    filled = round(BAR_WIDTH * t / N_test);
    bar    = [repmat('#', 1, filled), repmat('-', 1, BAR_WIDTH - filled)];
    fprintf([ERASE '  [%s] %3d%%'], bar, round(100 * t / N_test));

    q_rand = QLIM(:,1)' + rand(1,6) .* (QLIM(:,2)' - QLIM(:,1)');
    p_rand = xarm6_fk(q_rand);

    % Rotate through conventions each iteration to exercise all paths
    conv = mod(t-1, 4);
    switch conv
        case 0
            tgt = p_rand.T;
        case 1
            tgt = p_rand;
        case 2
            tgt = struct('p', p_rand.p, 'rpy', p_rand.rpy);
        case 3
            tgt = struct('p', p_rand.p, 'quat', p_rand.quat);
    end

    [Qt, ns] = xarm6_ik(tgt, q_rand, 'NSeeds', 8);
    if ns == 0
        no_sol = no_sol + 1;
        continue;
    end
    p_rec = xarm6_fk(Qt(1,:));
    pos_errs(t) = norm(p_rec.p - p_rand.p);
    dR = p_rec.T(1:3,1:3)' * p_rand.T(1:3,1:3);
    rot_errs(t) = rad2deg(acos(max(-1, min(1, (trace(dR)-1)/2))));
end
fprintf('\n');   % move to new line after progress bar

valid = ~isnan(pos_errs);
fprintf('  Configs tested      : %d\n', N_test);
fprintf('  No solution found   : %d  (%.1f%%)\n', no_sol, 100*no_sol/N_test);
fprintf('  Max position error  : %.4e mm\n', max(pos_errs(valid)));
fprintf('  Mean position error : %.4e mm\n', mean(pos_errs(valid)));
fprintf('  95th-pct pos error  : %.4e mm\n', prctile(pos_errs(valid), 95));
fprintf('  Max rotation error  : %.4e deg\n', max(rot_errs(valid)));
fprintf('  Mean rotation error : %.4e deg\n', mean(rot_errs(valid)));
fprintf('\nDone.\n');


% =========================================================================
%  Local functions  (must all appear after the last script statement)
% =========================================================================

function report_ik(label, Q, n, pose_true)
% Print IK solutions with position and rotation errors.
    fprintf('[%s]  solutions found: %d\n', label, n);
    for k = 1:n
        p_k     = xarm6_fk(Q(k,:));
        pos_err = norm(p_k.p - pose_true.p);
        dR      = p_k.T(1:3,1:3)' * pose_true.T(1:3,1:3);
        rot_err = rad2deg(acos(max(-1, min(1, (trace(dR)-1)/2))));
        fprintf('  Sol %d: pos_err=%.3e mm  rot_err=%.3e deg  q=[%s] deg\n', ...
            k, pos_err, rot_err, ...
            strjoin(arrayfun(@(x)sprintf('%.2f',x), rad2deg(Q(k,:)), ...
            'UniformOutput',false), ', '));
    end
    fprintf('\n');
end

function R = Rx(a)
    R = [1, 0, 0; 0, cos(a), -sin(a); 0, sin(a), cos(a)];
end

function R = Ry(a)
    R = [cos(a), 0, sin(a); 0, 1, 0; -sin(a), 0, cos(a)];
end

function R = Rz(a)
    R = [cos(a), -sin(a), 0; sin(a), cos(a), 0; 0, 0, 1];
end
