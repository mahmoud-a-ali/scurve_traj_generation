#!/usr/bin/env python
import math
import numpy as np
import traj 
import rospy


def synchronize_joint_motion( t_syn, pos_diff, v_start, v_end, abs_max_pos, abs_max_vel, abs_max_acc, abs_max_jrk):
	abs_v_start = abs(v_start)
	abs_v_end = abs(v_end)
	jm = abs_max_jrk
	## step 1: calculate all variables that determine which equation will be used for synchronization
	tj_2vf, ta_2vf, tj, ta, tv = traj.traj_segment_planning(0.0, pos_diff, abs_v_start, abs_v_end, abs_max_vel, abs_max_acc, abs_max_jrk)
	minPos_2vf, acc_2vf, tj_2vf, ta_2vf = traj.calculate_minPos_reachAcc_maxJrkTime_maxAccTime_to_final_vel( v_start, v_end, abs_max_vel, abs_max_acc, abs_max_jrk)


	##### assign new values for tj, ta, tv
	PDv   = pos_diff - minPos_2vf
	t_eqV = t_syn -  (2*tj_2vf + ta_2vf)
	tav = ta
	tvv = tv
	tjv = (t_eqV - 2*tav - tvv) / 4.0
	
	if tj == 0.0 and ta== 0.0:
		tjv = t_eqV / 4.0
		tav = 0.0
		tvv = 0.0

	

	case = 1
	rospy.logdebug(  ">> synchronize_jt_7phs case 1" )
	v = max(abs_v_start, abs_v_end)
	jk =  -(2*tav*v - PDv + 4*tjv*v + tvv*v)/(tav**2*tjv + 3*tav*tjv**2 + tvv*tav*tjv + 2*tjv**3 + tvv*tjv**2)
	a1 =  jk*tjv 
	a2 =  a1
	v1 =  jk*tjv*tjv/2 +             + v
	v2 =                    a1*tav   + v1
	v3 = -jk*tjv*tjv/2 +    a2*tjv   + v2
	rospy.logdebug( ">> jk, a1, v3:  {}, {}, {}".format( jk, a1, v3) )

	if abs(jk) > abs_max_jrk or v3 < 0.0 or v3 > abs_max_vel or abs(a1)> abs_max_acc:
		rospy.logdebug( ">> synchronize_jt_7phs case 2" )
		case = 2
		v = min(abs_v_start, abs_v_end)
		jk =  -(2*tav*v - PDv + 4*tjv*v + tvv*v)/(tav**2*tjv + 3*tav*tjv**2 + tvv*tav*tjv + 2*tjv**3 + tvv*tjv**2)
		a1 =  jk*tjv 
		a2 =  a1
		v1 =  jk*tjv*tjv/2 +             + v
		v2 =                    a1*tav   + v1
		v3 = -jk*tjv*tjv/2 +    a2*tjv   + v2
		rospy.logdebug( ">>  jk, a1, v3: {}, {}, {}".format( jk, a1, v3) )
		
		if abs(jk) > abs_max_jrk or v3 < 0.0 or v3 > abs_max_vel or abs(a1)> abs_max_acc:
			raise ValueError("synchronize_jt_7phs: motion is not feasible") 

	## caculate jrk_sign_dur according to case
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
	if v_start >= 0 and v_end >= 0 and pos_diff >=0:
		return 1 # 'positive_motion'
	elif v_start <= 0 and v_end <= 0 and pos_diff <=0:
		return -1 #'negative_motion'
	else:
		raise ValueError("identify_motion_direction: motion is not feasible")
		return 0 #'complex_motion'

def calculate_max_reachable_velocity_for_synchronized_time(PD, v_start, T):
	## seperate times to 3 variables
	Dv = (Dp - v_start*T)/T**2
	v_end = v_start + Dv
	return v_end











