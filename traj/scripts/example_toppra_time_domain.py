#!/usr/bin/env python
import math
import traj 
from matplotlib import pyplot as plt
import rospy
import numpy as np
import time

'''
example to calculate the max_reaxchable_vel per each dof starting with v_start to end with v_end (using t-domain)
currently: ndof are not time-synchronized yet
'''
  
#set log_level to debug
rospy.init_node('traj_segment_node')#, log_level=rospy.DEBUG)
t_init = time.time()


##### max values
abs_max_pos = 30.0
abs_max_vel = 3.0
abs_max_acc = 4.0
abs_max_jrk = 10.0



## select on of the following pathes
# Jon's path [the one Jon used for the first demo with zeros velocities]
positions =[]
positions.append( [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] )
positions.append( [ 1.5, 0.7, 0.3, 0.0, 0.0, 0.0] )
positions.append( [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] )
positions.append( [-1.5, 0.7, 0.3, 0.0, 0.0,0.0] )
positions.append( [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] )


##random traj 
positions =[]
positions.append( [ 0.0, 0.0, 0.0, 0.7, 0.0, 0.0] )
positions.append( [ 0.4, 0.2, 0.3, 0.4, 0.0, 0.0] )
positions.append( [ 0.8, 0.4, 0.6, 0.1, 0.0, 0.0] )
positions.append( [ 1.2, 0.6, 0.3, -.2, 0.0, 0.0] )
positions.append( [ 0.8, 0.4, 0.0, -.5, 0.0, 0.0] )


path_np = np.array( positions )
path = np.rot90( path_np ).tolist()
path.reverse()
 
### check number of joints, number of segments 
n_jts  = len(path)
n_wpts = len(path[0])
n_segs = n_wpts - 1
print "n_jts, n_wpts, n_segs :"
print n_jts, n_wpts, n_segs  


#start/end velocity for each joint 
v_start = [0.0 for pt in range (0, n_jts) ]
v_end   = [0.0 for pt in range (0, n_jts) ]



#####################  n-dof : to find max reachable vel per individual path seperately #######################
Estimated_vel = traj.find_max_estimated_vel_per_ndof_path(path, v_start, v_end, abs_max_pos, abs_max_vel, abs_max_acc, abs_max_jrk)
fig = plt.figure() 
for jt in range(0, n_jts ):   
    plt.plot( Estimated_vel[jt], label='jt_{}'.format(jt))
    plt.plot( Estimated_vel[jt], 'o')
plt.xlabel("waypoint_number")
plt.ylabel("velocity")
plt.legend()
plt.grid()
# plt.show()


### fit each segment with the non_zero segment planning according to 
### the estimated max velocity which has been calculated in the previous step
### then sample the segment with the sample function and plot it
POS = []
VEL = []
ACC = []
JRK = [] 
TIM = []
T_SEG= []

T_jt_seg_durs= []
J_jt_seg_durs= []
T_jt_seg_Dur= []
for jt in range(0, n_jts): 
    Pos = []
    Vel = []
    Acc = []
    Jrk = [] 
    Tim = [] 
    T_seg= []
    # time and sampling 
    t_start = 0.0
    frq = 125.0
    T_seg=[ t_start]
    # print "\n ###################### jt {} ###################### ".format(jt) 
    T_seg_durs = [] 
    J_seg_durs = [] 
    t_jt_seg_Dur = []
    for seg in range(0, n_segs):
        #print "\n>> seg {}: path[seg]={}, path[seg+1]={}, Estimated_vel[seg]={}, Estimated_vel[seg+1]={}".format(seg, path[i][seg], path[i][seg+1], Estimated_vel[i][seg], Estimated_vel[i][seg+1])
        jrk_dur = traj.calculate_jerk_sign_and_duration(path[jt][seg], path[jt][seg+1], Estimated_vel[jt][seg], Estimated_vel[jt][seg+1], abs_max_pos, abs_max_vel, abs_max_acc, abs_max_jrk)
        
        #convert duration and jrk to 1d list
        jrk_dur_arr = np.array( jrk_dur)
        T =  jrk_dur_arr[:,1].tolist()
        J =  jrk_dur_arr[:,0].tolist()
        # print "\n>>>> seg {}:  tot_seg_time = {}, [ph_jrk, ph_dur]\n{}".format(seg, sum(T), jrk_dur_arr) 
        T_seg_durs.append( T )
        J_seg_durs.append( J )
        t_jt_seg_Dur.append( sum(T) )
        #T_seg: contains the start/end time of each segment
        T_seg.append(  sum(T) + T_seg[seg] )
        t_start= T_seg[seg]
        t= t_start
        while t < T_seg[seg+1] :
            pos, vel, acc, jrk = traj.sample_segment(t, t_start, path[jt][seg], Estimated_vel[jt][seg], J, T )    
    
            Pos.append(pos)
            Vel.append(vel) #* math.copysign(1, vel))
            Acc.append(acc)
            Jrk.append(jrk)
            Tim.append(t )
            t = t + (1.0/frq)
    POS.append(Pos)
    VEL.append(Vel)
    ACC.append(Acc)
    JRK.append(Jrk)
    TIM.append(Tim)
    T_SEG.append(T_seg)

    T_jt_seg_durs.append( T_seg_durs )
    J_jt_seg_durs.append( J_seg_durs )
    T_jt_seg_Dur.append( t_jt_seg_Dur )
  
    
### plot pos, vel, acc, jrk. plot waypoints and estimated velocity as well to check if there is any difference 
fig, axes = plt.subplots(4,n_jts, sharex=True)

for jt in range(0, n_jts): 
    axes[0][jt].plot( TIM[jt], POS[jt])
    axes[0][jt].plot( T_SEG[jt], path[jt], 'o')
    axes[0][jt].grid()
axes[0][0].set_ylabel('position')

for jt in range(0, n_jts ):
    axes[1][jt].plot( TIM[jt], VEL[jt])
    axes[1][jt].plot( T_SEG[jt], Estimated_vel[jt],  'o')
    axes[1][jt].grid()
axes[1][0].set_ylabel('velocity')


for jt in range(0, n_jts):
    axes[2][jt].plot( TIM[jt], ACC[jt] )
    axes[2][jt].grid()
axes[2][0].set_ylabel('acceleration')


for jt in range(0, n_jts):
    axes[3][jt].plot( TIM[jt], JRK[jt] )
    axes[3][jt].grid()
    axes[3][jt].set_xlabel('Time')
axes[3][0].set_ylabel('jerk')



plt.legend()
plt.show()









