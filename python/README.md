## Quick start

### Python Example

```python
import numpy as np
from xarm6_fk import xarm6_fk
from xarm6_ik import xarm6_ik
from xarm6_plot import xarm6_plot
import matplotlib.pyplot as plt

# Forward kinematics
q    = np.deg2rad([30, -45, 0, 0, 45, -30])
pose = xarm6_fk(q)
print(f"Flange position : {pose.p.round(2)} mm")
print(f"TCP    position : {pose.p_tcp.round(2)} mm")
print(f"RPY (deg)       : {np.degrees(pose.rpy).round(2)}")
print(f"Quaternion      : {pose.quat.round(4)}")

# Inverse kinematics
Q, n = xarm6_ik(pose, q_ref=q)
print(f"\n{n} IK solution(s):")
print(np.round(np.degrees(Q), 2))

# 3-D visualisation
fig = xarm6_plot(q, show_frames=True)
plt.show()
```

## Configuring the tool (TCP and Bounding Box)

Edit the bottom of `xarm6_params.py` 

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

