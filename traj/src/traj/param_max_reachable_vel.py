#!/usr/bin/env python
"""
To calculates the maximum reachable velocity at the end of the segment based on the position difference (p_start and p_end) and the starting velocity (v_start), 
taking into considereation vel/acc/jrk constraints. this idea is the same idea behind the TOPP-RA paper: "A New Approach to Time-Optimal Path Parameterization
based on Reachability Analysis [H. pham 2018]", where they calculate max reachable velocity based on only the acc constraint, Here, we added the jerk constraint as well to calcuate 
the max reachable velocity at the end of the segment

to be done:
to add zero velocity whenever there is change in direction, then it works for positive and negative velocity as well 

paper link: https://www.researchgate.net/publication/318671280_A_New_Approach_to_Time-Optimal_Path_Parameterization_Based_on_Reachability_Analysis
"""
import rospy
import traj
    
def max_reachable_vel_per_segment(p_start, p_end, v_start, abs_max_pos, abs_max_vel, abs_max_acc, abs_max_jrk):
    """
    the maximum reachable velocity at the end of the segment, based on the position difference: 1. (p_end - p_start), 2.v_start
    this functions return the phase times: tj, ta, tv considering max three phases: [acc profile be like /`````\........ ] 
    a) tj: time to reach max  acc 
    b) ta: time to reach max vel
    c) tv: time with max vel to complete the required position difference p_end-p_start"""         

    
    rospy.logdebug( "\n max_vel_info: p_start={}, p_end={}, v_start ={} ".format(p_start, p_end, v_start) )
    pos_diff = p_end - p_start
    
    ############ A) if (pos_diff is zero), then time is zero and v_end = v_start ############
    if pos_diff == 0.0:
        rospy.logdebug("\n>>> case A")
        tj= 0.0
        ta= 0.0
        tv= 0.0
        v_end= v_start
        return tj, ta, tv, v_end, v_end/abs(v_end)
        
        
        
    
    ############ B) if (pos_diff and v0 have same sign), then (vf is +ve) and (vf is based on pos_diff)  ############
    elif (pos_diff > 0.0 and v_start >= 0.0):
        rospy.logdebug(  "\n>>> case B, +ve direction" )   
        vel_dir = 1
        tj, ta, tv, v_end = traj.max_reachable_vel(p_start, p_end, v_start, abs_max_pos, abs_max_vel, abs_max_acc, abs_max_jrk)
        return tj, ta, tv, v_end, vel_dir
   
     
    elif (pos_diff < 0.0 and v_start <= 0.0):
        vel_dir = -1
        rospy.logdebug(  "\n>>> case B, -ve direction" )   
        tj, ta, tv, v_end = traj.max_reachable_vel( p_end, p_start, -v_start, abs_max_pos, abs_max_vel, abs_max_acc, abs_max_jrk)
        return tj, ta, tv, -v_end, vel_dir
       
                   
            
            
    ############ C) if (pos_diff and v0 have different sign), then (vf is opposite to v0 sign) and (vf is based on both pos_diff_v0_0 and pos_diff_0_vf) ############
    
    elif (pos_diff > 0.0 and v_start < 0.0) or (pos_diff < 0.0 and v_start > 0.0):
        rospy.logdebug(  "\n>>> complex case: not implemented yet "  )      
        return 0.0, 0.0, 0.0, 0.0
        