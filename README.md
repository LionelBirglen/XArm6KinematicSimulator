# XArm6 Kinematic Simulator
Matlab and Python Files to Simulate the Kinematics of a UFactory xArm6 Robot Arm. Functions are provided for solving the forward kinematics, inverse kinematics, and 3-D visualisation of the [UFactory xArm 6](https://www.ufactory.cc/xarm-collaborative-robot) robotic arm,

![Matlab Plot Example](media/matlab_plot_example.png)
![Matlab Trajectory Example](media/matlab_trajectory_example.png)



# Features
- **Forward kinematics** returns the flange and TCP pose in four orientation conventions simultaneously: homogeneous matrix, roll-pitch-yaw (XYZ fixed-angle), unit quaternion, and axis-angle vector (UFactory convention).
- **Inverse kinematics** returns joint angles from a know pose using a numerical Jacobian damped-least-squares solver with multiple random seeds. Accepts target poses in any of the same four conventions. The solver accounts for the non-spherical wrist geometry (joint-5 offset of 76 mm).
- **3-D visualiser** — rectangular beam links, capped cylinders aligned to each joint's rotation axis, coordinate frame triads at the base and TCP by default, and an animated TCP trajectory example.
- **Configurable** with a single definition of DH parameters, joint limits, TCP offset+bounding box defined in "xarm6_params". Changes are automatically propagated to FK, IK, and the visualiser.
---

## File overview

| File | Purpose |
|---|---|
| `xarm6_params.py` / `.m` | DH table, joint limits, TCP offset, tool bounding box |
| `xarm6_fk.py` / `.m` | Forward kinematics → flange + TCP pose in all conventions |
| `xarm6_ik.py` / `.m` | Inverse kinematics — accepts T, RPY, quaternion, or axis-angle |
| `xarm6_plot.py` / `.m` | 3-D visualiser with optional TCP trajectory |
| `xarm6_kinematics_demo.py` / `.m` | FK/IK validation + Monte-Carlo accuracy test |
| `xarm6_plot_demo.py` / `.m` | Static pose viewer, IK verification, and animation |
| `xarm6_reload.py` | Reload all modules after editing `xarm6_params` (Python only) |

## Python Requirements
numpy >= 1.24
matplotlib >= 3.7

## MATLAB Requirements
nothing. Tested on Matlab R2023a.

---



### MATLAB

```matlab
q    = deg2rad([30, -45, 0, 0, 45, -30]);
pose = xarm6_fk(q);
fprintf('Flange : [%.2f, %.2f, %.2f] mm\n', pose.p);
fprintf('TCP    : [%.2f, %.2f, %.2f] mm\n', pose.p_tcp);

[Q, n] = xarm6_ik(pose, q);
fprintf('%d IK solution(s)\n', n);

xarm6_plot(q);
```

---

## Orientation conventions

All conventions follow the UFactory SDK definitions.

| Field | Convention |
|---|---|
| `rpy` | XYZ fixed-angle: R = Rz(yaw) · Ry(pitch) · Rx(roll) |
| `quat` | Unit quaternion [w, qx, qy, qz], w ≥ 0 |
| `axis_angle` | Compact rotation vector [Rx, Ry, Rz] = φ · [nx, ny, nz], φ ≥ 0 |

---

## Configuring the tool (TCP)

Edit the bottom of `xarm6_params.py` (or `.m`):

```python
# Translation of the TCP origin in flange coordinates [mm]
TCP_translation = np.array([0.0, 0.0, 50.0])

# Rotation of the TCP frame relative to the flange
TCP_rotation = np.eye(3)

# Tool bounding box in local TCP frame [mm]: [x_min, x_max, y_min, y_max, z_min, z_max]
# The visualiser draws the EE beam using this box.
tcp_box = np.array([-5.0, 5.0, -10.0, 10.0, -50.0, 0.0])
```

After editing, reload in Python without restarting the interpreter:

```python
exec(open('xarm6_reload.py').read())
```

---

## IK input conventions

`xarm6_ik` accepts any of the following:

```python
# 1. 4×4 homogeneous matrix
Q, n = xarm6_ik(pose.T)

# 2. Pose dataclass (direct output of xarm6_fk)
Q, n = xarm6_ik(pose)

# 3. Position + RPY
Q, n = xarm6_ik({'p': pose.p, 'rpy': pose.rpy})

# 4. Position + quaternion
Q, n = xarm6_ik({'p': pose.p, 'quat': pose.quat})

# 5. Position + axis-angle
Q, n = xarm6_ik({'p': pose.p, 'axis_angle': pose.axis_angle})
```

---

## Joint limits

| Joint | Lower | Upper |
|---|---|---|
| q1 | −360° | +360° |
| q2 | −118° | +118° |
| q3 | −225° | +11°  |
| q4 | −360° | +360° |
| q5 | −97°  | +180° |
| q6 | −360° | +360° |

---

## License

MIT
