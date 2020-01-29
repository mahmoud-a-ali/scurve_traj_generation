#!/usr/bin/env python
"""
Simple function to plot trajectory segment, given the start/end positions/velocities,
Start and end acceleration/jerk are assumed to be zero.
"""
from matplotlib import pyplot as plt
import traj

def plot_traj_segment(p_start, p_end, v_start, v_end, p_max, v_max, a_max, j_max):
    position, velocity, acceleration, jerk = traj.fit_traj_segment(p_start, p_end, v_start, v_end, p_max, v_max, a_max, j_max)
    traj.plot.plot_trajectory(plt, position, velocity, acceleration, jerk, n_points=100, v_max=v_max, a_max=a_max, j_max=j_max)
    plt.show()
    
