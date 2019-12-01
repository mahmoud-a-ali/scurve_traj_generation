#!/usr/bin/env python
"""
Simple example that parametrizes a 2d joint-space path.
"""
from matplotlib import pyplot as plt
import numpy as np

import traj

# Test path
path = np.array([(0.0, 0.0), (1.0, 1.0), (1.5, 0.8), (-0.2, 0.4)])

path_function = traj.parameterize_path(path)

# Plot sampled points along the parametrized path.
traj.plot.plot_2d_path(path_function, 100)

# Plot the waypoints in the original path for comparison.
plt.plot([q[0] for q in path], [q[1] for q in path], 'bx', label='original waypoints')

plt.legend()
plt.show()

