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
  
# set log_level to debug
rospy.init_node('traj_segment_node')#, log_level=rospy.DEBUG)
t_init = time.time()

# max values
abs_max_pos = 30.0
abs_max_vel = 3.0
abs_max_acc = 4.0
abs_max_jrk = 10.0

# select on of the following pathes
# Jon's path [the one Jon used for the first demo with zeros velocities]
positions =[]
positions.append( [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] )
positions.append( [ 1.5, 0.7, 0.3, 0.0, 0.0, 0.0] )
positions.append( [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] )
positions.append( [-1.5, 0.7, 0.3, 0.0, 0.0,0.0] )
positions.append( [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] )

#random traj 
positions =[]
positions.append( [ 0.0, 0.0, 0.0, 0.7, 0.0, 0.0] )
positions.append( [ 0.4, 0.2, 0.3, 0.4, 0.0, 0.0] )
positions.append( [ 0.8, 0.4, 0.6, 0.1, 0.0, 0.0] )
positions.append( [ 1.2, 0.6, 0.3, -.2, 0.0, 0.0] )
positions.append( [ 0.8, 0.4, 0.0, -.5, 0.0, 0.0] )

path_np = np.array( positions )
path = np.rot90( path_np ).tolist()
path.reverse()
 
# number of joints, number of segments 
n_jts  = len(path)
n_wpts = len(path[0])
n_segs = n_wpts - 1

# start/end velocity for each joint 
v_start = [0.0 for pt in range (0, n_jts) ]
v_end   = [0.0 for pt in range (0, n_jts) ]

# n-dof : to find max reachable vel per individual path seperately #######################
estimated_vel = traj.reachable_vel_at_each_waypoint_multi_dof_path_case(path, v_start, v_end, abs_max_pos, abs_max_vel, abs_max_acc, abs_max_jrk)
fig = plt.figure() 
for jt in range(0, n_jts ):   
    plt.plot( estimated_vel[jt], label='jt_{}'.format(jt))
    plt.plot( estimated_vel[jt], 'o')
plt.xlabel("waypoint_number")
plt.ylabel("velocity")
plt.legend()
plt.grid()
# plt.show()

# fit each segment with the non_zero segment planning according to 
# the estimated max velocity which has been calculated in the previous step
# then sample the segment with the sample function and plot it
traj_pos = []
traj_vel = []
traj_acc = []
traj_jrk = [] 
traj_time = []
jt_seg_times= []

t_jt_seg_durs= []
j_jt_seg_durs= []
t_jt_seg_dur= []
for jt in range(0, n_jts): 
    pos = []
    vel = []
    acc = []
    jrk = [] 
    jt_times = [] 
    seg_times= []
    # time and sampling 
    t_start = 0.0
    frq = 125.0
    seg_times=[ t_start]
    # print "\n >> jt_{}  ".format(jt) 
    t_seg_durs = [] 
    j_seg_durs = [] 
    t_jt_seg_dur = []
    for seg in range(0, n_segs):
        #print "\n>> seg {}: path[seg]={}, path[seg+1]={}, estimated_vel[seg]={}, estimated_vel[seg+1]={}".format(seg, path[i][seg], path[i][seg+1], estimated_vel[i][seg], estimated_vel[i][seg+1])
        jrk_dur = traj.calculate_jerk_sign_and_duration(path[jt][seg], path[jt][seg+1], estimated_vel[jt][seg], estimated_vel[jt][seg+1], abs_max_pos, abs_max_vel, abs_max_acc, abs_max_jrk)
        
        #convert duration and jrk to 1d list
        jrk_dur_arr = np.array( jrk_dur)
        phases_durs =  jrk_dur_arr[:,1].tolist()
        phases_jrks =  jrk_dur_arr[:,0].tolist()
        # print "\n>>>> seg {}:  tot_seg_time = {}, [ph_jrk, ph_dur]\n{}".format(seg, sum(T), jrk_dur_arr) 
        t_seg_durs.append( phases_durs )
        j_seg_durs.append( phases_jrks )
        t_jt_seg_dur.append( sum(phases_durs) )
        #T_seg: contains the start/end time of each segment
        seg_times.append(  sum(phases_durs) + seg_times[seg] )
        t_start= seg_times[seg]
        t= t_start
        while t < seg_times[seg+1] :
            ps, vl, ac, jk = traj.sample_segment(t, t_start, path[jt][seg], estimated_vel[jt][seg], phases_jrks, phases_durs )    
    
            pos.append(ps)
            vel.append(vl) 
            acc.append(ac)
            jrk.append(jk)
            jt_times.append(t )
            t = t + (1.0/frq)
    traj_pos.append(pos)
    traj_vel.append(vel)
    traj_acc.append(acc)
    traj_jrk.append(jrk)
    traj_time.append(jt_times)
    jt_seg_times.append(seg_times)

    t_jt_seg_durs.append( t_seg_durs )
    j_jt_seg_durs.append( j_seg_durs )
    t_jt_seg_dur.append( t_jt_seg_dur )
  
### plot pos, vel, acc, jrk. plot waypoints and estimated velocity as well to check if there is any difference 
fig, axes = plt.subplots(4,n_jts, sharex=True)
for jt in range(0, n_jts): 
    axes[0][jt].plot( traj_time[jt], traj_pos[jt])
    axes[0][jt].plot( jt_seg_times[jt], path[jt], 'o')
    axes[0][jt].grid()
axes[0][0].set_ylabel('position')

for jt in range(0, n_jts ):
    axes[1][jt].plot( traj_time[jt], traj_vel[jt])
    axes[1][jt].plot( jt_seg_times[jt], estimated_vel[jt],  'o')
    axes[1][jt].grid()
axes[1][0].set_ylabel('velocity')

for jt in range(0, n_jts):
    axes[2][jt].plot( traj_time[jt], traj_acc[jt] )
    axes[2][jt].grid()
axes[2][0].set_ylabel('acceleration')

for jt in range(0, n_jts):
    axes[3][jt].plot( traj_time[jt], traj_jrk[jt] )
    axes[3][jt].grid()
    axes[3][jt].set_xlabel('Time')
axes[3][0].set_ylabel('jerk')
plt.legend()
plt.show()
