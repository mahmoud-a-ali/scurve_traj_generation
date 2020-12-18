from matplotlib import pyplot as plt
import numpy as np
import traj

joint_colors = ['r', 'b', 'g', 'c', 'm', 'y', 'k', 'w']

"""
Plot a 1 dimensional trajectory. Plots position, velocity, acceleration, and jerk versus time
in separate subplots.
Uses the entire provided figure, adding subplots as needed.
"""
def plot_segment(figure, position, velocity, acceleration, jerk, n_points=500, j_max=None,
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
    plt.suptitle("traj_segment")




def fit_and_plot_segment(p_start, p_end, v_start, v_end, p_max, v_max, a_max, j_max):
    '''
    this function plots a trajectory segment, given the start/end positions/velocities,
    Start and end acceleration/jerk are assumed to be zero.
    '''
    position, velocity, acceleration, jerk = traj.fit_traj_segment(p_start, p_end, v_start, v_end, p_max, v_max, a_max, j_max)
    plot_segment(plt.figure(), position, velocity, acceleration, jerk, n_points=100, v_max=v_max, a_max=a_max, j_max=j_max)
    plt.show()







def find_phase(t, phase_times):
    '''
    this functions find the phase at which the time instant t belongs.
    phase_times: contains the start and end time of each phase  
    '''
    phase_times = np.asarray(phase_times)
    ph = np.searchsorted(phase_times, t, side='left') -1 
    return ph


def sample( t, p_start, v_start, phase_times, phase_jrk,  infl_points_acc, infl_points_vel, infl_points_pos):
    '''
    this functions is to sample a trajectory segment, it takes as argument:
        1. starting time "t_start", starting position "p_start", and starting velocity "v_start"
        2. jerk value and jerk duration of each phase: J: jerk value "Jrk_v", duration "T_ph"
        3. the time instant "t" at which we need to calculate pos, vel, acc, jrk 
        4. pos,vel, and acc at each inflection point where phase changes: Pos_v, Vel_v, Acc_v 
    it returns:
        pos, vel, acc, and jrk at that time instant "t"
    J, T are vectors of size equal to number of phases 
    '''
    ph = find_phase(t, phase_times)
    if ph < 0: #before segment
        jrk =  0.0
        acc =  infl_points_acc[0]
        vel =  infl_points_vel[0]
        pos =  infl_points_pos[0]
        return pos, vel, acc, jrk
        
    elif ph > 9: #after segment
        jrk =  0.0
        acc =  infl_points_acc[-1]
        vel =  infl_points_vel[-1]
        pos =  infl_points_pos[-1]
        return pos, vel, acc, jrk
        
    else:
        t = t - phase_times[ph]
        jrk =  phase_jrk[ph]
        acc =  phase_jrk[ph]*t        +  infl_points_acc[ph] 
        vel =  phase_jrk[ph]*t**2/2.0 +  infl_points_acc[ph]*t           + infl_points_vel[ph] 
        pos =  phase_jrk[ph]*t**3/6.0 +  infl_points_acc[ph]*t**2/2.0    + infl_points_vel[ph]*t + infl_points_pos[ph] 
        return pos, vel, acc, jrk
  
  
def sample_segment(t, t_start, p_start, v_start, phase_jrk, phase_dur):
    '''
    this functions is to sample a trajectory segment, it takes as argument:
        1. starting time "t_start", starting position "p_start", and starting velocity "v_start"
        2. jerk value and jerk duration of each phase: J: jerk value "J", duration "T"
        3. the time instant "t" at which we need to calculate pos, vel, acc, jrk 
    it returns:
        pos, vel, acc, and jrk at that time instant "t"

    J, T are vectors of size equal to number of phases 
    '''
    #convert durations to  times
    phase_times = [ sum(phase_dur[0:i]) for i in range(0, len(phase_dur)+1) ]
       
    #calculate pos,vel,acc at each inflection point where phase changes: 
    infl_points_acc=[0.0]
    infl_points_vel=[v_start]
    infl_points_pos=[p_start]
    for i in range (0, 10):
        infl_points_acc.append( phase_jrk[i]*phase_dur[i]        +  infl_points_acc[i] )
        infl_points_vel.append( phase_jrk[i]*phase_dur[i]**2/2.0 +  infl_points_acc[i]*phase_dur[i]          + infl_points_vel[i]  )
        infl_points_pos.append( phase_jrk[i]*phase_dur[i]**3/6.0 +  infl_points_acc[i]*phase_dur[i]**2/2.0   + infl_points_vel[i]*phase_dur[i]  + infl_points_pos[i] )
    
    return sample( t-t_start, p_start, v_start, phase_times, phase_jrk,  infl_points_acc, infl_points_vel, infl_points_pos)
