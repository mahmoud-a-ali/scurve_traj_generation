#!/usr/bin/env python
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


###choose/uncomment one of the following cases
# starting/ending sotion for each joint
P_start =[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
P_end   =[7.7, 1.9, 5.1, 1.5, 7.3, 3.4]
PD_vec = [pf-pi for pf, pi in zip(P_end, P_start)]
n_jts = len( PD_vec)


## case_1: zero velocity
# V_start = [0.0 for jt in range(n_jts)]
# V_end = [0.0 for jt in range(n_jts)]


## case_2: all joints starting/ending velocities are equal 
# V_start = [1.2 for jt in range(n_jts)]
# V_end   = [0.7 for jt in range(n_jts)]


## case_3: joints starting/ending velocities are different 
V_start = [3.0, 0.2, 0.8, 0.5, 2.5, 1.6]
V_end   = [3.0, 0.8, 0.8, 0.5, 1.0, 1.5]


## case_4: random values for position difference and start/ending velocity
# import random 
# n_jts = 6
# P_start = [0.0 for jt in range(n_jts)]
# PD_vec  = np.random.uniform(low=-abs_max_pos, high=abs_max_pos, size=(n_jts,) )
# P_end   = [ p0+pd for p0, pd in zip(P_start, PD_vec)]

# V_start = np.random.uniform(low=0.0, high=abs_max_vel, size=(n_jts,))
# V_end = np.random.uniform(low=0.0, high=abs_max_vel, size=(n_jts,))

# V_start = [math.copysign(1, PD_vec[jt])*V_start[jt] for jt in range(n_jts) ]
# V_end   = [math.copysign(1, PD_vec[jt])*V_end[jt] for jt in range(n_jts) ]




motion_dir= [ ] 
for jt in range( n_jts ):
	motion_dir.append( traj.motion_direction(V_start[jt],  V_end[jt], PD_vec[jt])  )



## step 1:
min_T  = [ ]
for jt in range( n_jts ):
	## min time for each segment: phases times
	tj_2vf, ta_2vf, t_jrk, t_acc, t_vel = traj.traj_segment_planning(0.0, abs( PD_vec[jt]), abs( V_start[jt]), abs( V_end[jt]) ,  abs_max_vel, abs_max_acc, abs_max_jrk)
	min_t = 2*tj_2vf + ta_2vf +  4*t_jrk + 2*t_acc + t_vel
	min_T.append(  min_t  ) 


ref_jt = min_T.index(max(min_T))
syn_t  = max(min_T) 
rospy.logdebug( ">> syn_t : {} ".format(  syn_t  )  )
rospy.logdebug( ">> ref_jt: {} ".format(  ref_jt )  )
rospy.logdebug( ">> min_T: {} ".format(  min_T   )  )


### step 3: calculate new jrk_sgn_dur
Dur_jt = []
Jrk_jt = []
for jt in range( n_jts ):
	rospy.logdebug( "\n\n>> jt:{}, PD: {}, v_start:{}, v_end:{}".format(  jt , PD_vec[jt], V_start[jt], V_end[jt] ) )
	pos_diff = abs( PD_vec[jt]  )   
	v_start = abs( V_start[jt]  )
	v_end   = abs( V_end[jt]  )
	if jt == ref_jt:
		jrk_sign_dur = traj.calculate_jerk_sign_and_duration(0.0, pos_diff, v_start, v_end , abs_max_pos, abs_max_vel, abs_max_acc, abs_max_jrk)
	
	else:
		jrk_sign_dur = traj.synchronize_joint_motion( syn_t, pos_diff, v_start, v_end , abs_max_pos, abs_max_vel, abs_max_acc, abs_max_jrk)



	dur = [jsd[1] for jsd in jrk_sign_dur]
	jrk = [motion_dir[jt]*jsd[0] for jsd in jrk_sign_dur]

	Dur_jt.append( dur  )
	Jrk_jt.append( jrk  )

	rospy.logdebug( ">> dur:{}".format( sum(dur) )   )


### sampling and plotting
frq = 125.0
t_start = 0.0
t = t_start
POS = [ [] for jt in range(n_jts)]
VEL = [ [] for jt in range(n_jts)]
ACC = [ [] for jt in range(n_jts)]
JRK = [ [] for jt in range(n_jts)]
Tim = [ ]


while t <= syn_t:
	for jt in range( n_jts ):
		p_start = P_start[jt]
		v_start = V_start[jt]
		T = Dur_jt[jt]
		J = Jrk_jt[jt]
		pos, vel, acc, jrk = traj.sample_segment(t, t_start, p_start, v_start, J, T)
		POS[jt].append(  pos  )
		VEL[jt].append(  vel  )
		ACC[jt].append(  acc  )
		JRK[jt].append(  jrk  )	
	Tim.append( t )
	t = t + 1/frq
	

### plot pos, vel, acc, jrk. plot waypoints and estimated velocity as well to check if there is any difference 
T_start = [Tim[0] for jt in range(n_jts)]
T_end = [Tim[-1] for jt in range(n_jts)]

fig, axes = plt.subplots(4, sharex=True)
for jt in range(0, n_jts): 
    axes[0].plot( Tim, POS[jt])
    axes[1].plot( Tim, VEL[jt])
    axes[2].plot( Tim, ACC[jt] )
    axes[3].plot( Tim, JRK[jt] )

axes[0].plot( T_start, P_start, '*')
axes[0].plot( T_end,   P_end,   '*')

axes[1].plot( T_start, V_start, '*')
axes[1].plot( T_end,   V_end,   '*')


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