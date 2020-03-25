#!/usr/bin/env python
import math
import numpy as np
import traj 
import rospy


def synchronize_joint_motion(t_syn, pos_diff, v_start, v_end, abs_max_pos, abs_max_vel, abs_max_acc, abs_max_jrk):
	'''
	this function selects a motion profile for  a general trajectory segment considering the total motion time of the segment is "t_syn"
	it returns the jerk_value && duration associated with each phase of the segment
	it raise an error if:
		1. if the given time "t_syn" is less than the minimum time caclulated using the maximum jerk 
		2. if the given position differnce "pos_diff" is reached before the final velocity "v_end" can be reached  
		3. if the combination of the position difference"pos_diff", the final velocity "v_end", and the motion time "t_syn" 
			gives non-monotonic motion 

	this function is based on the same idea of the paper entitled : 
	Online Trajectory Generation: Basic Concepts for Instantaneous Reactions to Unforeseen Events[1], 
	section V,  synchronization steps 1,2,3


	[1] https://www-cs.stanford.edu/groups/manips/publications/pdfs/Kroeger_2010_TRO.pdf
	'''

	abs_v_start = abs(v_start)
	abs_v_end = abs(v_end)
	jm = abs_max_jrk

	# calculate all variables that determine which equation will be used for synchronization
	tj_2vf, ta_2vf, tj, ta, tv = traj.traj_segment_planning(0.0, pos_diff, abs_v_start, abs_v_end, abs_max_vel, abs_max_acc, abs_max_jrk)
	min_pos_2vf, acc_2vf, tj_2vf, ta_2vf = traj.calculate_min_pos_reached_acc_jrk_time_acc_time_to_reach_final_vel( v_start, v_end, abs_max_vel, abs_max_acc, abs_max_jrk)

	# assign new values for tj, ta, tv
	pd_eq_vel   = pos_diff - min_pos_2vf
	t_eq_vel = t_syn -  (2*tj_2vf + ta_2vf)
	tav = ta
	tvv = tv
	tjv = (t_eq_vel - 2*tav - tvv) / 4.0
	
	if tj == 0.0 and ta== 0.0:
		tjv = t_eq_vel / 4.0
		tav = 0.0
		tvv = 0.0

	# choose a motion profile 
	case = 1
	rospy.logdebug(">> synchronize_jt_7phs case 1")
	v = max(abs_v_start, abs_v_end)
	jk =  -(2*tav*v - pd_eq_vel + 4*tjv*v + tvv*v)/(tav**2*tjv + 3*tav*tjv**2 + tvv*tav*tjv + 2*tjv**3 + tvv*tjv**2)
	a1 =  jk*tjv 
	a2 =  a1
	v1 =  jk*tjv*tjv/2 +             + v
	v2 =                    a1*tav   + v1
	v3 = -jk*tjv*tjv/2 +    a2*tjv   + v2
	rospy.logdebug(">> jk, a1, v3:  {}, {}, {}".format( jk, a1, v3))

	if abs(jk) > abs_max_jrk or v3 < 0.0 or v3 > abs_max_vel or abs(a1)> abs_max_acc:
		rospy.logdebug( ">> synchronize_jt_7phs case 2" )
		case = 2
		v = min(abs_v_start, abs_v_end)
		jk =  -(2*tav*v - pd_eq_vel + 4*tjv*v + tvv*v)/(tav**2*tjv + 3*tav*tjv**2 + tvv*tav*tjv + 2*tjv**3 + tvv*tjv**2)
		a1 =  jk*tjv 
		a2 =  a1
		v1 =  jk*tjv*tjv/2 +             + v
		v2 =                    a1*tav   + v1
		v3 = -jk*tjv*tjv/2 +    a2*tjv   + v2
		rospy.logdebug(">>  jk, a1, v3: {}, {}, {}".format( jk, a1, v3))
		
		if abs(jk) > abs_max_jrk or v3 < 0.0 or v3 > abs_max_vel or abs(a1)> abs_max_acc:
			raise ValueError("synchronize_jt_7phs: motion is not feasible") 

	# caculate jrk_sign_dur according to case
	if case == 1:
		if abs_v_end < abs_v_start:
			jrk_sgn_dur = [(jk, tjv), (0.0, tav), (-jk, tjv), (0.0, tvv), (-jk,tjv), (0.0, tav), (jk, tjv),
			                (-jm, tj_2vf), (0.0, ta_2vf), (jm, tj_2vf)]

		else:
			jrk_sgn_dur = [(jm, tj_2vf), (0.0, ta_2vf), (-jm, tj_2vf), 
						   (jk, tjv), (0.0, tav), (-jk, tjv), (0.0, tvv), (-jk,tjv), (0.0, tav), (jk, tjv)]

	elif case == 2:
		if abs_v_end > abs_v_start:
			jrk_sgn_dur = [(jk, tjv), (0.0, tav), (-jk, tjv), (0.0, tvv), (-jk,tjv), (0.0, tav), (jk, tjv),
			               (jm, tj_2vf), (0.0, ta_2vf), (-jm, tj_2vf)]

		else:
			jrk_sgn_dur = [(-jm, tj_2vf), (0.0, ta_2vf), (jm, tj_2vf), 
							(jk, tjv), (0.0, tav), (-jk, tjv), (0.0, tvv), (-jk,tjv), (0.0, tav), (jk, tjv)]

	return jrk_sgn_dur 



def motion_direction( v_start, v_end, pos_diff):
	''' 
	this function checks the direction of the motion based on the starting/ending velocity and the position difference
	if the position differnce is not aligned with the direction of the starting/ending velocity it raises an error 
	'''
	# positive_motion_case:
	if v_start >= 0 and v_end >= 0 and pos_diff >=0:
		return 1
	# negative_motion_case: 
	elif v_start <= 0 and v_end <= 0 and pos_diff <=0:
		return -1 
	# complex_motion_case:
	else:
		raise ValueError("identify_motion_direction: motion is not feasible")
		return 0 












