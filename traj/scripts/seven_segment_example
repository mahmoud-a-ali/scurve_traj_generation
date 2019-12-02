#!/usr/bin/env python
"""
Simple example that computes a seven segment motion profile a 1d motion with given start and
end positions. Start and end velocity/acceleration/jerk are assumed to be zero.
"""
from matplotlib import pyplot as plt
import numpy as np

import traj

j_max = 0.1
a_max = 0.4
v_max = 2.0
p_start = 0.0
p_end = 30.0

position, velocity, acceleration, jerk = traj.fit_seven_segment(
        p_start, p_end, v_max, a_max, j_max)
traj.plot.plot_trajectory(position, velocity, acceleration, jerk, n_points=100,
        v_max=v_max, a_max=a_max, j_max=j_max)
plt.show()

