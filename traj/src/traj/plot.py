from matplotlib import pyplot as plt

def plot_trajectory(trajectory, n_points=1000, j_max=None, a_max=None, v_max=None):
    plot_times = np.linspace(trajectory.segment_boundary_times[0],
            trajectory.segment_boundary_times[-1], 1000)
    positions = np.array([trajectory.pos(t) for t in plot_times])
    velocities = np.array([trajectory.vel(t) for t in plot_times])
    accelerations = np.array([trajectory.acc(t) for t in plot_times])
    jerks = np.array([trajectory.jerk(t) for t in plot_times])
    fig, axes = plt.subplots(4)
    axes[0].plot(plot_times, positions)
    axes[1].plot(plot_times, velocities)
    if v_max is not None:
        axes[1].plot(plot_times, [v_max] * len(plot_times), '--', color='red')
        axes[1].plot(plot_times, [-v_max] * len(plot_times), '--', color='red')
    axes[2].plot(plot_times, accelerations)
    if a_max is not None:
        axes[2].plot(plot_times, [a_max] * len(plot_times), '--', color='red')
        axes[2].plot(plot_times, [-a_max] * len(plot_times), '--', color='red')
    axes[3].plot(plot_times, jerks)
    if j_max is not None:
        axes[3].plot(plot_times, [j_max] * len(plot_times), '--', color='red')
        axes[3].plot(plot_times, [-j_max] * len(plot_times), '--', color='red')
    plt.show()

def plot_2d_path(boundaries, functions, symbol, npoints):
    S, path_points = sample_path(boundaries, functions, symbol, npoints)
    plt.plot(path_points[:,0], path_points[:,1], 'r.-')
