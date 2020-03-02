from matplotlib import pyplot as plt
import numpy as np

joint_colors = ['r', 'b', 'g', 'c', 'm', 'y', 'k', 'w']

"""
Plot a 1 dimensional trajectory. Plots position, velocity, acceleration, and jerk versus time
in separate subplots.
Uses the entire provided figure, adding subplots as needed.
"""
def plot_trajectory(figure, position, velocity, acceleration, jerk, n_points=300, j_max=None,
        a_max=None, v_max=None, p_max=None):
    boundaries = jerk.boundaries
    plot_times = np.linspace(position.boundaries[0], position.boundaries[-1], n_points)
    positions = np.array([position(t) for t in plot_times])
    velocities = np.array([velocity(t) for t in plot_times])
    accelerations = np.array([acceleration(t) for t in plot_times])
    jerks = np.array([jerk(t) for t in plot_times])
    axes = figure.subplots(4, sharex=True)
    for joint_i in range(positions.shape[1]):
        c = joint_colors[joint_i]
        axes[0].plot(plot_times, positions[:,joint_i], c=c)
        axes[0].set_ylabel('position')
        axes[1].plot(plot_times, velocities[:,joint_i], c=c)
        axes[1].set_ylabel('velocity')
        axes[2].plot(plot_times, accelerations[:,joint_i], c=c)
        axes[2].set_ylabel('acceleration')
        axes[3].plot(plot_times, jerks[:,joint_i], c=c)
        axes[3].set_ylabel('jerk')

    axes[0].vlines(boundaries, positions.min(), positions.max(), color=(0.8, 0.8, 0.8))

    if p_max is not None:
        axes[0].plot(plot_times, [p_max] * len(plot_times), '--', color='red')
        axes[0].plot(plot_times, [-p_max] * len(plot_times), '--', color='red')

    if v_max is not None:
        axes[1].plot(plot_times, [v_max] * len(plot_times), '--', color='red')
        axes[1].plot(plot_times, [-v_max] * len(plot_times), '--', color='red')
        axes[1].vlines(boundaries, -v_max, v_max, color=(0.8, 0.8, 0.8))

    if a_max is not None:
        axes[2].plot(plot_times, [a_max] * len(plot_times), '--', color='red')
        axes[2].plot(plot_times, [-a_max] * len(plot_times), '--', color='red')
        axes[2].vlines(boundaries, -a_max, a_max, color=(0.8, 0.8, 0.8))

    if j_max is not None:
        axes[3].plot(plot_times, [j_max] * len(plot_times), '--', color='red')
        axes[3].plot(plot_times, [-j_max] * len(plot_times), '--', color='red')
        axes[3].vlines(boundaries, -j_max, j_max, color=(0.8, 0.8, 0.8))
    plt.legend()

"""
Plot the positions for a 2 dimensional path on the provided matplotlib axes.
"""
def plot_2d_path(axes, piecewise_function, npoints, linewidth=1.0, label='path points'):
    """
    Plot a 2d path that is represented as a piecewise function of a single variable.
    """
    S, path_points = piecewise_function.sample(npoints)
    axes.plot(path_points[:,0], path_points[:,1], 'r.-', linewidth=linewidth, label=label)
    axes.set_title('Joint space path')
    axes.set_xlabel('Joint 1')
    axes.set_ylabel('Joint 2')
    axes.set_ylabel('Joint 2')

