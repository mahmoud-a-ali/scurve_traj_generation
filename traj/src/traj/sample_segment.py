#!/usr/bin/env python
"""
sampling funtion for 10_phase segment, it takes as argument:
1. start time/pos/vel
2. jerk value and duration of each phase
3. the time instant at which we need to calculate pos, vel, acc, jrk
and returns pos, vel, acc, jrk at that time instant
"""

import numpy as np


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
