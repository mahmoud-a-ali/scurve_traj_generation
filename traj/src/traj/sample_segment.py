#!/usr/bin/env python
"""
sampling funtion for 10_phase segment, it takes as argument:
1. start time/pos/vel
2. jerk value and duration of each phase
3. the time instant at which we need to calculate pos, vel, acc, jrk
and returns pos, vel, acc, jrk at that time instant
"""

import numpy as np
from matplotlib import pyplot as plt


def find_phase(t, T_ph):
    T_ph = np.asarray(T_ph)
    ph = np.searchsorted(T_ph, t, side='left') -1 
    return ph
    


def sample( t, p_start, v_start, T_ph, Jrk_v,  Acc_v, Vel_v, Pos_v):    
    ph = find_phase(t, T_ph)
    if ph < 0: #before segment
        jrk =  0.0
        acc =  Acc_v[0]
        vel =  Vel_v[0]
        pos =  Pos_v[0]
        return pos, vel, acc, jrk
        
    elif ph > 9: #after segment
        jrk =  0.0
        acc =  Acc_v[-1]
        vel =  Vel_v[-1]
        pos =  Pos_v[-1]
        return pos, vel, acc, jrk
        
    else:
        t = t - T_ph[ph]
        jrk =  Jrk_v[ph]
        acc =  Jrk_v[ph]*t        +  Acc_v[ph] 
        vel =  Jrk_v[ph]*t**2/2.0 +  Acc_v[ph]*t           + Vel_v[ph] 
        pos =  Jrk_v[ph]*t**3/6.0 +  Acc_v[ph]*t**2/2.0    + Vel_v[ph]*t + Pos_v[ph] 
        return pos, vel, acc, jrk
  
  
  
  
def sample_segment(t, t_start, p_start, v_start, ph_jrk_dur ):
    #convet duration and jrk to 1d list
    jrk_dur_arr = np.array( ph_jrk_dur)
    J =  jrk_dur_arr[:,0].tolist()
    T =  jrk_dur_arr[:,1].tolist()
    
    #convert dur to  times
    T_ph = [ sum(T[0:i]) for i in range(0, len(T)+1) ]
       
    #calculate pos,vel,acc at each phase change (inflection points): 
    A=[0.0]
    V=[v_start]
    P=[p_start]
    for i in range (0, 10):
        A.append( J[i]*T[i]        +  A[i] )
        V.append( J[i]*T[i]**2/2.0 +  A[i]*T[i]          + V[i]  )
        P.append( J[i]*T[i]**3/6.0 +  A[i]*T[i]**2/2.0   + V[i]*T[i]  + P[i] )
    
    return sample( t-t_start, p_start, v_start, T_ph, J,  A, V, P)

        
#
##test case
#t_jrk=01.0
#t_acc=01.0
#t_vel=0.50
#
#t_jrk_to_vf= 0.5
#t_acc_to_vf=0.5
#
#p_start= 15.0
#v_start= 2.0
#
#j_max = 5
#j_max_to_vf = -5
#jrk_dur = [(j_max, t_jrk), (0.0, t_acc), (-j_max, t_jrk), (0.0, t_vel), (-j_max,t_jrk), (0.0, t_acc), (j_max, t_jrk),    
#                               (j_max_to_vf, t_jrk_to_vf), (0.0, t_acc_to_vf), (-j_max_to_vf, t_jrk_to_vf)]
#
##j_max = 5
##j_max_to_vf = 5
##jrk_dur = [(j_max_to_vf, t_jrk_to_vf), (0.0, t_acc_to_vf), (-j_max_to_vf, t_jrk_to_vf),
##           (j_max, t_jrk), (0.0, t_acc), (-j_max, t_jrk), (0.0, t_vel), (-j_max,t_jrk), (0.0, t_acc), (j_max, t_jrk)]
#
#
#
#
#
#
#
#
#Tim = []
#POS = []
#VEL = []
#ACC = []
#JRK = []
## time and sampling 
#t = 0.0
#t_start = 0.0
#frq = 125.0
#
#
#while t < 10 :
#    pos, vel, acc, jrk = sample_segment(t, t_start, p_start, v_start, jrk_dur )
#    
#      
#    Tim.append(t)
#    POS.append(pos)
#    VEL.append(vel)
#    ACC.append(acc)
#    JRK.append(jrk)
#    t = t + (1.0/frq)
##
#fig, axes = plt.subplots(4, sharex=True)
#axes[0].plot(Tim, POS)
#axes[1].plot(Tim, VEL)
#axes[2].plot(Tim, ACC)
#axes[3].plot(Tim, JRK)
#
#plt.show()
#
#
#
#
#
#
#
#
