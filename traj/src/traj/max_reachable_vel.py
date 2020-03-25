#!/usr/bin/env python
import rospy
import math
import traj
    
    
def calculate_min_pos_reached_acc_jrk_time_acc_time_to_reach_max_vel_3phases_case(abs_v_start, vm, am, jm):
    '''
    This function calculates the following variables to reach the absolute Maximum Velocity "vm"  starting with initial_vel "v_start":
    1. the minimum position required to reach the maximum absolute velocity "vm"
    2. the acceleration "acc" that has been reached to reach the maximum absolute velocity starting with "v_start"
    3. phases times to reach maximum absolute velocity: jerk_phase time "tj" and acceleration_phase time "ta" 
    '''
    p0=0.0
    v0=abs_v_start
    a0=0.0   
    ar = math.sqrt( jm*abs(vm-v0) ) #only three phases  
    tj= ar/jm
    ta=0.0
    if v0==vm:
        return 0,0,0,0
    else:
        jm = math.copysign(jm,  (vm-v0) ) #here is always +ve 

    t=tj
    a1 =  jm*t +  a0
    a2 =          a1
    v1 =  jm*t*t/2.0 +  a0*t   + v0
    v2 =                a1*ta  + v1
    p1 =  jm*t*t*t/6.0 +  a0*t*t/2.0    + v0*t  + p0
    p2 =                  a1*ta*ta/2.0  + v1*ta + p1
    p3 = -jm*t*t*t/6.0 +  a2*t*t/2.0    + v2*t  + p2
   
    if abs(a2)<=abs(am):
        acc_to_max_vel = a2 
        min_pos_to_max_vel = p3 
    else:
        tj =abs(am/jm) 
        ta= ( abs(vm-v0) - abs((am**2/jm)) )/am        
        t = tj
        a1 =  jm*t +  a0
        a2 =          a1
        v1 =  jm*t*t/2.0 +  a0*t   + v0
        v2 =                a1*ta  + v1
        p1 =  jm*t*t*t/6.0 +  a0*t*t/2.0    + v0*t  + p0
        p2 =                  a1*ta*ta/2.0  + v1*ta + p1
        p3 = -jm*t*t*t/6.0 +  a2*t*t/2.0    + v2*t  + p2
        acc_to_max_vel = a2 
        min_pos_to_max_vel = p3 
    return min_pos_to_max_vel, acc_to_max_vel, tj, ta
         

def calculate_min_pos_to_reach_max_acc_3phases_case(v0, vm, am, jm):
    '''
    This function calculates the minimum position required to reach the absolute Maximum acceleration "am"  starting with initial_vel "v0":
    '''
    p0=0.0
    a0=0.0
    tj= am/jm    
    t=tj
    ta= 0.0
    a1 =  jm*t +  a0
    a2 =          a1
    v1 =  jm*t*t/2.0 +  a0*t   + v0
    v2 =                a1*ta  + v1
    p1 =  jm*t*t*t/6.0 +  a0*t*t/2.0    + v0*t  + p0
    p2 =                  a1*ta*ta/2.0  + v1*ta + p1
    p3 = -jm*t*t*t/6.0 +  a2*t*t/2.0    + v2*t  + p2
    min_pos_to_max_acc = p3
    return min_pos_to_max_acc
             
         
def calculate_acc_time_final_vel_for_pos_diff_3phases_case(Dp, abs_v_start, vm, am, jm):
    '''
    This function calculates:
    1. the acceleration_phase time "ta" which is required to move the joint 
        to new position equal to the starting position plus a position difference "DP"
    2. the final velocity "v_end" when the the joint reaches the final position "current position + position difference DP " 
    '''
    # here we wont reach max vel but still we reach max acc,  which means no const_vel phase     
    p0=0.0
    v0=abs_v_start
    a0=0.0

    #p3_eq:  2*am^3 + 3*am^2*jm*ta + am*jm^2*ta^2 + 4*v0*am*jm + 2*v0*jm^2*ta + 2*p0*jm^2 -DP*2*jm^2 = 0.0
    a= am*jm**2
    b= 3*am**2*jm + 2*v0*jm**2
    c= 2*am**3 + 4*v0*am*jm + p0*jm**2 - 2*jm**2*Dp
    rt1, rt2, n_rts= traj.quad_eq_real_root (a, b, c)
    if n_rts ==0:
        raise ValueError("there is no real positive roots!" )
    elif n_rts==1:
        ta = rt1
    elif n_rts ==2:
        ta = traj.min_positive_root2(rt1, rt2)
  
    t= am/jm
    a1 =  jm*t +  a0;
    a2 =          a1;
    v1 =  jm*t*t/2.0 +  a0*t   + v0
    v2 =                a1*ta  + v1
    v3 = -jm*t*t/2.0 +  a2*t   + v2
    max_acc_time = ta
    v_end = v3
    return max_acc_time, v_end
    
    
def calculate_jrk_time_reached_acc_final_vel_for_pos_diff_3phases_case(Dp, abs_v_start, vm, am, jm):
    '''
    This function calculates:
    1. the acceleration_phase time "ta" which is required to move the joint 
        to new position equal to the starting position plus a position difference equal to DP
    2. the final velocity "v_end" when the the joint reaches the final position "current position + position difference DP " 
    '''

    p0 = 0.0
    v0 = abs_v_start   
    a0= 0.0
   #p3_eq: ar^3 + 2*v0*ar*jm + p0*jm^2  -DP*jm^2 = 0.0    
    a= 1.0
    b= 0.0
    c= 2*v0*jm
    d= p0*jm**2 - Dp*jm**2
    rt1, rt2, rt3, n_rts= traj.real_roots_cubic_eq (a, b, c, d )

    if n_rts ==0:
        raise ValueError("there is no real positive roots!" )
    elif n_rts==1:
        ar = rt1
    elif n_rts ==2:
        ar = traj.min_positive_root2(rt1, rt2)
    elif n_rts ==3:
        ar = traj.min_positive_root3(rt1, rt2, rt3)
  
    tj= ar/jm
    ta= 0.0
    t=tj
    a1 =  jm*t +  a0
    a2 =          a1
    v1 =  jm*t*t/2.0 +  a0*t   + v0
    v2 =                a1*ta  + v1
    v3 = -jm*t*t/2.0 +  a2*t   + v2
    reached_Acc = a2
    v_end = v3 
    return tj, reached_Acc, v_end
    
    
def max_reachable_vel_per_segment(abs_pos_diff, abs_v_start, abs_max_pos, abs_max_vel, abs_max_acc, abs_max_jrk):
    '''
    this function calculates the maximum reachable velocity at the end of the segment, based on the position difference (p_end - p_start), 
    and the starting velocity "v_start"
    it returns the phases' times: jerk_phase time "tj", acceleration_phase time "ta", velocity_phase time "tv" 
    considering a three phases motion: [acc profile be like /`````\........ ] 
    '''         
    rospy.logdebug( "\n max_vel_info: pos_diff={}, v_start ={} ".format( abs_pos_diff, abs_v_start) )
    # A) if (pos_diff is zero), then time is zero and v_end = v_start 
    if abs_pos_diff == 0.0:
        rospy.logdebug("\n>>> case A")
        tj= 0.0
        ta= 0.0
        tv= 0.0
        abs_v_end= abs_v_start
        return tj, ta, tv, abs_v_end

    # B) if (pos_diff and v0 have same sign), then (vf is +ve) and (vf is based on pos_diff)  
    elif abs_pos_diff > 0.0 and abs_v_start >= 0.0:  
        rospy.logdebug(  "\n>>> case B")   
        min_pos_to_max_vel, acc_to_max_vel, tj, ta = calculate_min_pos_reached_acc_jrk_time_acc_time_to_reach_max_vel_3phases_case(abs_v_start, abs_max_vel, abs_max_acc, abs_max_jrk)
        rospy.logdebug( "min_pos_to_max_vel= {}, acc_to_max_vel={}, tj={}, ta={}".format(min_pos_to_max_vel, acc_to_max_vel, tj, ta) )
        
        # B1) if abs_pos_diff >= min_pos_to_max_vel, then reach to vm and then continue with vm
        if abs_pos_diff >= min_pos_to_max_vel:
            rospy.logdebug( "\n>>> case B1")
            abs_v_end = abs_max_vel
            tv = (abs_pos_diff - min_pos_to_max_vel) / abs_max_vel
            return tj, ta, tv, abs_v_end
        
        # B2) else abs_pos_diff < min_pos_to_max_vel, and acc_to_max_vel >= abs_max_acc
        else:
            rospy.logdebug( "\n>>> case B2")
            tv= 0.0
            min_pos_to_max_acc = calculate_min_pos_to_reach_max_acc_3phases_case(abs_v_start, abs_max_vel, abs_max_acc, abs_max_jrk)
            rospy.logdebug( "min_pos_to_max_acc= {}".format(min_pos_to_max_acc) )
        
            # B2a) if abs_pos_diff >= min_pos_to_max_acc, then calculate ta such that it gives  abs_pos_diff, tj is already known: tj= am/jm 
            if abs_pos_diff >= min_pos_to_max_acc:
                tj= abs_max_acc/abs_max_jrk
                rospy.logdebug( ">>>case B2a" )
                ta, abs_v_end = calculate_acc_time_final_vel_for_pos_diff_3phases_case(abs_pos_diff, abs_v_start, abs_max_vel, abs_max_acc, abs_max_jrk)
                rospy.logdebug( "tj={},  ta={},  tv={},     v_end= {}".format( tj, ta, tv,  abs_v_end ) )
                return tj, ta, tv, abs_v_end
                
            # B2b) else abs_pos_diff < min_pos_to_max_acc, then calculate tj such that it gives  abs_pos_diff, ta is already known: ta= 0.0  
            else:
                ta= 0.0
                rospy.logdebug( ">>>case B2b" )
                tj, reached_Acc, abs_v_end = calculate_jrk_time_reached_acc_final_vel_for_pos_diff_3phases_case(abs_pos_diff, abs_v_start, abs_max_vel, abs_max_acc, abs_max_jrk)
                rospy.logdebug(  "tj={},  ta={},  tv={},   reached_Acc={},     v_end={}".format( tj, ta, tv, reached_Acc, abs_v_end ) )
                return tj, ta, tv, abs_v_end
            
    # C) if (pos_diff and v0 have different sign), then (vf is opposite to v0 sign) and (vf is based on both pos_diff_v0_0 and pos_diff_0_vf) 
    elif abs_pos_diff < 0.0 or abs_v_start < 0.0:#(pos_diff > 0.0 and v_start < 0.0) or (pos_diff < 0.0 and v_start > 0.0):
        rospy.logdebug(  "\n>>> complex case: not implemented yet "  )      
        raise ValueError("Case C: in param_max_vel" )
        return 0.0, 0.0, 0.0, 0.0
