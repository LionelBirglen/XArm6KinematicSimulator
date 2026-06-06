## Quick start

### Matlab

```matlab
q    = deg2rad([30, -45, 0, 0, 45, -30]);
pose = xarm6_fk(q);
fprintf('Flange : [%.2f, %.2f, %.2f] mm\n', pose.p);
fprintf('TCP    : [%.2f, %.2f, %.2f] mm\n', pose.p_tcp);

[Q, n] = xarm6_ik(pose, q);
fprintf('%d IK solution(s)\n', n);

xarm6_plot(q);
```
## Configuring the tool (TCP and Bounding Box)

Edit the bottom of `xarm6_params.m` 

```matlab
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
```

After editing, relaunch any function you wish to use the updated parameters.

## IK input conventions

`xarm6_ik` accepts any of the following:

```matlab
       %% 1. From a homogeneous matrix (original usage, unchanged)
       q_true = deg2rad([30, -45, 0, 0, 45, -30]);
       pose   = xarm6_fk(q_true);
       [Q, n] = xarm6_ik(pose.T, q_true);

       %% 2. From a pose struct (direct output of xarm6_fk)
       [Q, n] = xarm6_ik(pose, q_true);

       %% 3. From position + RPY
       tgt.p   = [150; 80; 300];           % mm
       tgt.rpy = deg2rad([10; -20; 45]);   % [roll; pitch; yaw] rad
       [Q, n]  = xarm6_ik(tgt);

       %% 4. From position + quaternion
       tgt.p    = [150; 80; 300];
       tgt.quat = [0.924; 0.0; 0.383; 0.0];  % [w; qx; qy; qz]
       [Q, n]   = xarm6_ik(tgt);

       %% 5. From position + UFactory axis-angle
       tgt.p          = [150; 80; 300];
       tgt.axis_angle = [1.57; 0; 0];   % phi*[nx;ny;nz]
       [Q, n]         = xarm6_ik(tgt);
```
