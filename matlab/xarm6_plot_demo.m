% xarm6_plot_demo.m
% Script to demonstrate xarm6_plot for static poses and a smooth animation.
%
% Requires: xarm6_fk.m, xarm6_ik.m, xarm6_plot.m, xarm6_params.m
%
% Author: Lionel Birglen (with assistance from Anthropic Claude)
% Polytechnique Montreal, 2026

clear; clc; close all;

% ---- Shared axis limits and floor size (mm) used across all panels ------
AXES_LIM     = [-600 600  -500 500  -80 900];
FLOOR_LENGTH = 1200;
FLOOR_WIDTH  = 1000;

% --- 1. Static views of several named poses ------------------------------
poses = { ...
    zeros(1,6),                               'Home (all zeros)' ; ...
    deg2rad([  0, -60,   0,   0,   0,   0]),  'Arm Up'           ; ...
    deg2rad([ 45, -45,   0,   0,  45, -30]),  'Sample Pose A'    ; ...
    deg2rad([-60, -30, -45,  20, -60,  90]),  'Sample Pose B'    ; ...
};

figure('Name','xArm6 - Static Poses', 'NumberTitle','off');

n = size(poses, 1);
l = round(sqrt(n));
c = ceil(n / round(sqrt(n)));
for k = 1:n
    ax = subplot(l, c, k);
    opt              = struct();
    opt.ax           = ax;
    opt.show_frames  = (k == 1);
    opt.title_str    = poses{k, 2};
    opt.axes_lim     = AXES_LIM;
    opt.floor_length = FLOOR_LENGTH;
    opt.floor_width  = FLOOR_WIDTH;
    xarm6_plot(poses{k, 1}, opt);
end

% --- 2. IK verification --------------------------------------------------
fprintf('\n--- IK round-trip visual check ---\n');
q_true    = deg2rad([30, -50, 0, 15, 40, -25]);
pose_true = xarm6_fk(q_true);

[Q_ik, n_sols] = xarm6_ik(pose_true, q_true);
fprintf('IK returned %d solution(s).\n', n_sols);

if n_sols > 0
    figure('Name','xArm6 - IK Verification', 'NumberTitle','off');

    ax1 = subplot(1, 2, 1);
    opt1              = struct();
    opt1.ax           = ax1;
    opt1.show_frames  = true;
    opt1.title_str    = 'Target (FK of q_true)';
    opt1.axes_lim     = AXES_LIM;
    opt1.floor_length = FLOOR_LENGTH;
    opt1.floor_width  = FLOOR_WIDTH;
    xarm6_plot(q_true, opt1);

    ax2 = subplot(1, 2, 2);
    opt2              = struct();
    opt2.ax           = ax2;
    opt2.show_frames  = true;
    opt2.title_str    = sprintf('Best IK Solution (of %d)', n_sols);
    opt2.axes_lim     = AXES_LIM;
    opt2.floor_length = FLOOR_LENGTH;
    opt2.floor_width  = FLOOR_WIDTH;
    xarm6_plot(Q_ik(1,:), opt2);

    pose_rec = xarm6_fk(Q_ik(1,:));
    fprintf('Position error of best IK solution: %.4e mm\n', ...
            norm(pose_rec.p - pose_true.p));
end

% --- 3. Smooth animation -------------------------------------------------
% questdlg is not available in Octave; use input() instead.
ans_str = input('Run a smooth animation? (~5 seconds) [y/N]: ', 's');
if isempty(ans_str), ans_str = 'N'; end

if strcmpi(ans_str, 'y')

    % ---- Optional: record the animation to an MP4 file -----------------
    rec_str = input('Record animation to MP4 file? [y/N]: ', 's');
    if isempty(rec_str), rec_str = 'N'; end
    record_movie = strcmpi(rec_str, 'y');

    is_octave = (exist('OCTAVE_VERSION', 'builtin') ~= 0);
    if record_movie && is_octave
        % Octave can write MP4 via the Octave Forge 'video' package, which
        % provides a VideoWriter class compatible with MATLAB's (requires
        % ffmpeg/libav and `pkg install -forge video`, then `pkg load video`).
        try
            pkg load video
        catch
            warning(['Recording requires the Octave Forge ''video'' package. ' ...
                     'Install it with: pkg install -forge video\n' ...
                     'Skipping video recording.']);
            record_movie = false;
        end
    end

    if record_movie
        out_name = input('Output filename (without extension) [xarm6_animation]: ', 's');
        if isempty(out_name), out_name = 'xarm6_animation'; end
        if is_octave
            % Octave's VideoWriter (Octave Forge 'video' package) infers
            % the codec from the file extension; it does not accept a
            % MATLAB-style profile name ('MPEG-4') as the second argument.
            v = VideoWriter([out_name '.mp4']);
        else
            v = VideoWriter([out_name '.mp4'], 'MPEG-4');
        end
        v.FrameRate = 25;
        if ~is_octave
            % Quality property is MATLAB-only; Octave's video package
            % (Octave Forge) does not expose it.
            v.Quality = 90;
        end
        open(v);
    end

    fig_anim = figure('Name','xArm6 - Animation', 'NumberTitle','off');
    ax_anim  = axes(fig_anim);

    N_frames = 120;
    t_vec    = linspace(0, 2*pi, N_frames);

    [~, qlim] = xarm6_params();

    opt_a              = struct();
    opt_a.ax           = ax_anim;
    opt_a.show_frames  = false;
    opt_a.axes_lim     = AXES_LIM;
    opt_a.floor_length = FLOOR_LENGTH;
    opt_a.floor_width  = FLOOR_WIDTH;

    % Pre-compute TCP trajectory
    tcp_traj = zeros(3, N_frames);
    for k = 1:N_frames
        t = t_vec(k);
        q = [  0.6*sin(t), ...
              -0.5 + 0.3*sin(2*t), ...
               min(0.0, -0.2 + 0.15*cos(t)), ...
               0.4*sin(3*t), ...
               0.5 + 0.3*sin(t+1), ...
               0.6*cos(2*t) ];
        q = max(q, qlim(:,1)');
        q = min(q, qlim(:,2)');
        tcp_traj(:, k) = xarm6_fk(q).p_tcp;
    end

    TRAJ_COLOR = [0.85 0.20 0.20];

    for k = 1:N_frames
        t = t_vec(k);
        q = [  0.6*sin(t), ...
              -0.5 + 0.3*sin(2*t), ...
               min(0.0, -0.2 + 0.15*cos(t)), ...
               0.4*sin(3*t), ...
               0.5 + 0.3*sin(t+1), ...
               0.6*cos(2*t) ];
        q = max(q, qlim(:,1)');
        q = min(q, qlim(:,2)');

        xarm6_plot(q, opt_a);
        hold on;

        plot3(ax_anim, tcp_traj(1,1:k), tcp_traj(2,1:k), tcp_traj(3,1:k), ...
              '-', 'Color', TRAJ_COLOR, 'LineWidth', 2.0);

        drawnow;

        if record_movie
            frame = getframe(fig_anim);
            writeVideo(v, frame);
        else
            pause(0.04);
        end
    end

    if record_movie
        close(v);
        fprintf('Movie saved to %s.mp4\n', out_name);
    end

    fprintf('Animation complete.\n');
end

fprintf('\nDone.\n');