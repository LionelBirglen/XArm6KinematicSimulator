"""
xarm6_reload.py
===============
Reloads all xArm 6 modules in dependency order so that any edits to
xarm6_params.py (TCP offset, tool box, joint limits, etc.) are picked
up immediately in the current Python session without restarting.

Usage (type in your interpreter or script):
    exec(open('xarm6_reload.py').read())

Author : Lionel Birglen (with assistance from Anthropic Claude)
Polytechnique Montreal, 2026
"""

import importlib
import xarm6_params, xarm6_fk, xarm6_ik, xarm6_plot

importlib.reload(xarm6_params)   # must come first — others depend on it
importlib.reload(xarm6_fk)
importlib.reload(xarm6_ik)
importlib.reload(xarm6_plot)

from xarm6_params import xarm6_params
from xarm6_fk    import xarm6_fk
from xarm6_ik    import xarm6_ik
from xarm6_plot  import xarm6_plot

print("xArm 6 modules reloaded.")
