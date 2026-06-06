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
