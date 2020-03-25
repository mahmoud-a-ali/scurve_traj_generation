#!/usr/bin/env python
'''
example for the joint motion synchronization algorithm 
'''
import numpy as np
import math
import traj
from matplotlib import pyplot as plt
import rospy

rospy.init_node('segment_synchronization', log_level=rospy.DEBUG)

#### limits:
abs_max_pos= 10.0 
abs_max_vel= 3.0
abs_max_acc= 4.0
abs_max_jrk= 10.0  


# starting/ending postion for each joint
pos_start =[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
pos_end   =[7.7, 1.9, 5.1, 1.5, 7.3, 3.4]
pos_diff = [pf-pi for pf, pi in zip(pos_end, pos_start)]
n_jts = len( pos_diff)

# choose/uncomment one of the following cases
# case_1: zero velocity
vel_start = [0.0 for jt in range(n_jts)]
vel_end = [0.0 for jt in range(n_jts)]

# case_2: all joints starting/ending velocities are equal 
vel_start = [1.2 for jt in range(n_jts)]
vel_end   = [0.7 for jt in range(n_jts)]

# case_3: joints starting/ending velocities are different 
vel_start = [3.0, 0.2, 0.8, 0.5, 2.5, 1.6]
vel_end   = [3.0, 0.8, 0.8, 0.5, 1.0, 1.5]

# case_4: random values for position difference and start/ending velocities
# import random 
# n_jts = 6
# pos_start = [0.0 for jt in range(n_jts)]
# pos_diff = np.random.uniform(low=-abs_max_pos, high=abs_max_pos, size=(n_jts,) )
# pos_end = [ p0+pd for p0, pd in zip(pos_start, pos_diff)]

# vel_start = np.random.uniform(low=0.0, high=abs_max_vel, size=(n_jts,))
# vel_end = np.random.uniform(low=0.0, high=abs_max_vel, size=(n_jts,))

# vel_start = [math.copysign(1, pos_diff[jt])*vel_start[jt] for jt in range(n_jts) ]
# vel_end = [math.copysign(1, pos_diff[jt])*vel_end[jt] for jt in range(n_jts) ]

motion_dir= [ ] 
for jt in range( n_jts ):
	motion_dir.append( traj.motion_direction(vel_start[jt],  vel_end[jt], pos_diff[jt])  )

# step 1: find the minimum time motion for each joints 
min_motion_time  = [ ]
for jt in range( n_jts ):
	## min time for each segment: phases times
	tj_2vf, ta_2vf, t_jrk, t_acc, t_vel = traj.traj_segment_planning(0.0, abs( pos_diff[jt]), abs( vel_start[jt]), abs( vel_end[jt]) ,  abs_max_vel, abs_max_acc, abs_max_jrk)
	min_time = 2*tj_2vf + ta_2vf +  4*t_jrk + 2*t_acc + t_vel
	min_motion_time.append(  min_time  ) 

# step 2: find the joint that has the maximum time motion (reference joint)
ref_jt = min_motion_time.index(max(min_motion_time))
syn_t  = max(min_motion_time) 
rospy.logdebug( ">> syn_t : {} ".format(  syn_t  )  )
rospy.logdebug( ">> ref_jt: {} ".format(  ref_jt )  )
rospy.logdebug( ">> min_T: {} ".format(  min_motion_time   )  )

# step 3: calculate new jrk_sgn_dur
jt_dur = []
jt_jrk = []
for jt in range( n_jts ):
	rospy.logdebug( "\n\n>> jt:{}, PD: {}, v_start:{}, v_end:{}".format(  jt , pos_diff[jt], vel_start[jt], vel_end[jt] ) )
	p_diff = abs( pos_diff[jt]  )   
	v_start = abs( vel_start[jt]  )
	v_end   = abs( vel_end[jt]  )
	
	if jt == ref_jt:
		jrk_sign_dur = traj.calculate_jerk_sign_and_duration(0.0, p_diff, v_start, v_end , abs_max_pos, abs_max_vel, abs_max_acc, abs_max_jrk)
	
	else:
		jrk_sign_dur = traj.synchronize_joint_motion( syn_t, p_diff, v_start, v_end , abs_max_pos, abs_max_vel, abs_max_acc, abs_max_jrk)

	dur = [jsd[1] for jsd in jrk_sign_dur]
	jrk = [motion_dir[jt]*jsd[0] for jsd in jrk_sign_dur]
	jt_dur.append( dur  )
	jt_jrk.append( jrk  )
	rospy.logdebug( ">> dur:{}".format( sum(dur) )   )

# sampling and plotting
frq = 125.0
t_start = 0.0
t = t_start
traj_pos = [ [] for jt in range(n_jts)]
traj_vel = [ [] for jt in range(n_jts)]
traj_acc = [ [] for jt in range(n_jts)]
traj_jrk = [ [] for jt in range(n_jts)]
traj_time = [ ]

while t <= syn_t:
	for jt in range( n_jts ):
		p_start = pos_start[jt]
		v_start = vel_start[jt]
		phases_dur = jt_dur[jt]
		phases_jrk = jt_jrk[jt]
		pos, vel, acc, jrk = traj.sample_segment(t, t_start, p_start, v_start, phases_jrk, phases_dur)
		traj_pos[jt].append(  pos  )
		traj_vel[jt].append(  vel  )
		traj_acc[jt].append(  acc  )
		traj_jrk[jt].append(  jrk  )	
	traj_time.append( t )
	t = t + 1/frq

# plot pos, vel, acc, jrk. plot waypoints and estimated velocity as well to check if there is any difference 
t_start_vec = [traj_time[0] for jt in range(n_jts)]
t_end_vec = [traj_time[-1] for jt in range(n_jts)]

fig, axes = plt.subplots(4, sharex=True)
for jt in range(0, n_jts): 
    axes[0].plot( traj_time, traj_pos[jt])
    axes[1].plot( traj_time, traj_vel[jt])
    axes[2].plot( traj_time, traj_acc[jt] )
    axes[3].plot( traj_time, traj_jrk[jt] )
axes[0].plot( t_start_vec, pos_start, '*')
axes[0].plot( t_end_vec,   pos_end,   '*')
axes[1].plot( t_start_vec, vel_start, '*')
axes[1].plot( t_end_vec,   vel_end,   '*')
axes[0].grid()
axes[1].grid()
axes[2].grid()
axes[3].grid()
axes[0].set_ylabel('position')
axes[1].set_ylabel('velocity')
axes[2].set_ylabel('acceleration')
axes[3].set_ylabel('jerk')
axes[3].set_xlabel('Time')
plt.legend()
plt.show()