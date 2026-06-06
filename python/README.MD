## Quick start

### Python

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
