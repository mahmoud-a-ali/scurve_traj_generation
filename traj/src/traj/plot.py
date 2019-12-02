from matplotlib import pyplot as plt
import numpy as np

def plot_trajectory(position, velocity, acceleration, jerk, n_points=1000, j_max=None,
        a_max=None, v_max=None):
    plot_times = np.linspace(position.boundaries[0], position.boundaries[-1], 1000)
    positions = np.array([position(t) for t in plot_times])
    velocities = np.array([velocity(t) for t in plot_times])
    accelerations = np.array([acceleration(t) for t in plot_times])
    jerks = np.array([jerk(t) for t in plot_times])
    fig, axes = plt.subplots(4)
    axes[0].plot(plot_times, positions)
    axes[0].set_ylabel('position')
    axes[1].plot(plot_times, velocities)
    axes[1].set_ylabel('velocity')
    if v_max is not None:
        axes[1].plot(plot_times, [v_max] * len(plot_times), '--', color='red')
        axes[1].plot(plot_times, [-v_max] * len(plot_times), '--', color='red')
    axes[2].plot(plot_times, accelerations)
    axes[2].set_ylabel('acceleration')
    if a_max is not None:
        axes[2].plot(plot_times, [a_max] * len(plot_times), '--', color='red')
        axes[2].plot(plot_times, [-a_max] * len(plot_times), '--', color='red')
    axes[3].plot(plot_times, jerks)
    axes[3].set_ylabel('jerk')
    if j_max is not None:
        axes[3].plot(plot_times, [j_max] * len(plot_times), '--', color='red')
        axes[3].plot(plot_times, [-j_max] * len(plot_times), '--', color='red')

def plot_2d_path(piecewise_function, npoints, label='path points'):
    """
    Plot a 2d path that is represented as a piecewise function of a single variable.
    """
    S, path_points = piecewise_function.sample(npoints)
    plt.plot(path_points[:,0], path_points[:,1], 'r.-', label=label)
    plt.title('Joint space path')
    plt.xlabel('Joint 1')
    plt.ylabel('Joint 2')
