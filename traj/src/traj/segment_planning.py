#!/usr/bin/env python
"""
this file contains main low level planning function "traj_segment_planning" to to calculate the values of t_jrk, t_acc, t_vel for each phase of the segment
"""
import rospy
import math
import cubic_eq_roots as rt


def calculate_min_pos_reached_acc_to_reach_max_vel(v, vm, am, jm):
    '''
    This function calculates the following variables to reach the absolute Maximum Velocity "vm"  starting with initial_velocity "v":
    1. the minimum position required to reach the maximum absolute velocity "vm"
    2. the acceleration "acc" that has been reached to reach the maximum absolute velocity starting with "v"
    '''
    p0=0.0
    a0=0.0
    v0=v
    ar = math.sqrt( jm*(vm-v0) )
    t= ar/jm
    ta=0.0
    tv=0.0
     
    a1 =  jm*t +  a0;
    a2 =          a1;
    a3 = -jm*t +  a2;
    a4 =          a3;
    a5 = -jm*t +  a4;
    a6 =          a5;
    
    v1 =  jm*t*t/2.0 +  a0*t   + v0;
    v2 =                a1*ta  + v1;
    v3 = -jm*t*t/2.0 +  a2*t   + v2;
    v4 =                a3*tv  + v3;
    v5 = -jm*t*t/2.0 +  a4*t   + v4;
    v6 =                a5*ta  + v5;

    p1 =  jm*t*t*t/6.0 +  a0*t*t/2.0    + v0*t  + p0;
    p2 =                  a1*ta*ta/2.0  + v1*ta + p1;
    p3 = -jm*t*t*t/6.0 +  a2*t*t/2.0    + v2*t  + p2;
    p4 =                  a3*t*t/2.0    + v3*tv + p3;
    p5 = -jm*t*t*t/6.0 +  a4*t*t/2.0    + v4*t  + p4;
    p6 =                  a5*ta*ta/2.0  + v5*ta + p5;
    p7 =  jm*t*t*t/6.0 +  a6*t*t/2.0    + v6*t  + p6;
    
    min_pos_to_max_vel = p7
    reached_acc_to_max_vel = ar
    return min_pos_to_max_vel, reached_acc_to_max_vel
    


def calculate_reached_acc_in_case_no_const_acc_phase(Dp, v, vm, am ,jm):
    '''
    This function calculates the acceleration that has been reached to move the joint a position equals to "DP" starting with velocity "v"
    this considers the case where there is no a constant acceleration phase.
    '''
    p0=0.0
    v0=v
     
    a=2.0
    b=0.0
    c=4*v0*jm 
    d=p0*jm**2-jm**2*Dp

    rt1, rt2, rt3, n_rts = rt.real_roots_cubic_eq ( a,  b,  c,  d)

    if n_rts==1:
        ar = rt1
    elif n_rts ==2:
        ar = rt.min_positive_root2(rt1, rt2)
    elif n_rts ==3:
        ar = rt.min_positive_root3(rt1, rt2, rt3)
 
    return ar
 


def calculate_min_pos_const_acc_time_to_reach_max_acc_and_max_vel(v, vm, am, jm):
    '''
    This function calculates the following variables to reach the absolute Maximum Velocity "vm"  
    starting with initial_velocity "v" considering that the absolute maximum acceleration "am" will be reached:
       1. the minimum position required to reach the maximum absolute velocity "vm"
       2. the acceleration_phase time "ta" to reach the maximum absolute velocity "vm"
    '''
    p0=0.0
    a0=0.0
    v0=v
    t = am/jm
    tv= 0.0
    ta= ( vm-v0-(am**2/jm) )/am

    a1 =  jm*t +  a0;
    a2 =          a1;
    a3 = -jm*t +  a2;
    a4 =          a3;
    a5 = -jm*t +  a4;
    a6 =          a5;
    
    v1 =  jm*t*t/2.0 +  a0*t   + v0;
    v2 =                a1*ta  + v1;
    v3 = -jm*t*t/2.0 +  a2*t   + v2;
    v4 =                a3*tv  + v3;
    v5 = -jm*t*t/2.0 +  a4*t   + v4;
    v6 =                a5*ta  + v5;
    
    p1 =  jm*t*t*t/6.0 +  a0*t*t/2.0    + v0*t  + p0;
    p2 =                  a1*ta*ta/2.0  + v1*ta + p1;
    p3 = -jm*t*t*t/6.0 +  a2*t*t/2.0    + v2*t  + p2;
    p4 =                  a3*t*t/2.0    + v3*tv + p3;
    p5 = -jm*t*t*t/6.0 +  a4*t*t/2.0    + v4*t  + p4;
    p6 =                  a5*ta*ta/2.0  + v5*ta + p5;
    p7 =  jm*t*t*t/6.0 +  a6*t*t/2.0    + v6*t  + p6;
    
    min_pos_to_max_vel= p7-p0
    t_max_acc = ta
    return min_pos_to_max_vel, t_max_acc



def calculate_min_pos_to_reach_max_acc(v, vm, am, jm):
    '''
    This function calculates the minimum position required to reach the absolute maximum acceleration "am", starting with initial velocity "v"
    '''
    p0=0.0
    a0=0.0
    v0=v
    t = am/jm
    tv= 0.0
    ta= 0.0 
    
    a1 =  jm*t +  a0;
    a2 =          a1;
    a3 = -jm*t +  a2;
    a4 =          a3;
    a5 = -jm*t +  a4;
    a6 =          a5;

    v1 =  jm*t*t/2.0 +  a0*t   + v0;
    v2 =                a1*ta  + v1;
    v3 = -jm*t*t/2.0 +  a2*t   + v2;
    v4 =                a3*tv  + v3;
    v5 = -jm*t*t/2.0 +  a4*t   + v4;
    v6 =                a5*ta  + v5;

    p1 =  jm*t*t*t/6.0 +  a0*t*t/2.0    + v0*t  + p0;
    p2 =                  a1*ta*ta/2.0  + v1*ta + p1;
    p3 = -jm*t*t*t/6.0 +  a2*t*t/2.0    + v2*t  + p2;
    p4 =                  a3*t*t/2.0    + v3*tv + p3;
    p5 = -jm*t*t*t/6.0 +  a4*t*t/2.0    + v4*t  + p4;
    p6 =                  a5*ta*ta/2.0  + v5*ta + p5;
    p7 =  jm*t*t*t/6.0 +  a6*t*t/2.0    + v6*t  + p6;
    
    min_pos_to_max_acc= p7-p0
    return min_pos_to_max_acc
    


def calculate_const_acc_time(Dp, v, vm, am, jm):
    '''
    This function calculates the acceleration_phase time "ta" which is required to move the joint 
    to new position equal to the current position plus a position difference "DP" starting with initial velocity "v"
    '''

    # here we wont reach max vel but still we reach max acc     
    p0=0.0
    v0=v
    #P7 equation is: (2*am^3 + 3*am^2*jm*ta + am*jm^2*ta^2 + 4*v0*am*jm + 2*v0*jm^2*ta + p0*jm^2)/jm^2
    a=0.0
    b= am*jm**2
    c=3*am**2*jm + 2*v0*jm**2
    d=2*am**3 + 4*v0*am*jm + p0*jm**2 - jm**2*Dp  
    rt1, rt2, rt3, n_rts = rt.real_roots_cubic_eq ( a,  b,  c,  d)

    if n_rts==1:
        ta = rt1
    elif n_rts ==2:
        ta = rt.min_positive_root2(rt1, rt2)
    elif n_rts ==3:
        ta = rt.min_positive_root3(rt1, rt2, rt3)    
    const_acc_time = ta
    return const_acc_time
    
    


def calculate_min_pos_reached_acc_jrk_time_acc_time_to_reach_final_vel(v0, vf, vm, am, jm):
    '''
    This function calculates the following variables to reach the final Velocity "vf"  starting with initial_vel "v0":
    1. the minimum position required to reach the final Velocity "vf"
    2. the acceleration "acc" that has been reached to reach the final Velocity "vf" starting with "v0"
    3. phases times to reach the final Velocity "vf": jerk_phase time "tj" and acceleration_phase time "ta" 
    '''
    p0=0.0
    a0=0.0

    ar = math.sqrt( jm*abs(vf-v0) )
    tj= ar/jm
    ta=0.0

    t=tj
    if vf==v0:
        return 0,0,0,0
    else:
        jm = math.copysign(jm,  (vf-v0) )
    
    a1 =  jm*t +  a0;
    a2 =          a1;

    v1 =  jm*t*t/2.0 +  a0*t   + v0;
    v2 =                a1*ta  + v1;

    p1 =  jm*t*t*t/6.0 +  a0*t*t/2.0    + v0*t  + p0;
    p2 =                  a1*ta*ta/2.0  + v1*ta + p1;
    p3 = -jm*t*t*t/6.0 +  a2*t*t/2.0    + v2*t  + p2;
  
    
    if abs(a2)<=abs(am):
        acc_to_vf = a2 #math.copysign(a2,  (vf-v0) )
        min_pos_to_vf = p3 #math.copysign(p3,  (vf-v0) )
    else:
        tj =abs(am/jm) 
        ta= ( abs(vf-v0) - abs((am**2/jm)) )/am        
        
        t = tj
        a1 =  jm*t +  a0;
        a2 =          a1;

        v1 =  jm*t*t/2.0 +  a0*t   + v0;
        v2 =                a1*ta  + v1;

        p1 =  jm*t*t*t/6.0 +  a0*t*t/2.0    + v0*t  + p0;
        p2 =                  a1*ta*ta/2.0  + v1*ta + p1;
        p3 = -jm*t*t*t/6.0 +  a2*t*t/2.0    + v2*t  + p2;  
        acc_to_vf = a2 #math.copysign(a2,  (vf-v0) )
        min_pos_to_vf = p3 #math.copysign(p3,  (vf-v0) )
    return min_pos_to_vf, acc_to_vf, tj, ta
         
         
         

    

def equal_vel_case_planning (pos_diff, v, abs_max_vel, abs_max_acc, abs_max_jrk):
    '''
    this function selects a motion profile for a trajectory segment in case of the final veoclity equals the starting velocity, both equal "v" 
    '''

    # check if a const_acc phase is required or not by calcuating the reached_acc_to_max_vel to reach vf
    min_pos_to_max_vel, reached_acc_to_max_vel = calculate_min_pos_reached_acc_to_reach_max_vel(v, abs_max_vel, abs_max_acc, abs_max_jrk)

    # check if reached_acc_to_max_vel < abs_max_acc: 
    #if yes: then no const_acc phase, check if a const_vel phase is required or not (to satisfy pos_diff) 
    if(reached_acc_to_max_vel<= abs_max_acc):
        rospy.logdebug("case a: maxAcc won't be reached !  /\\/ ")
        reached_vel = abs_max_vel
        reached_acc = reached_acc_to_max_vel      
        t_max_jrk = reached_acc_to_max_vel/abs_max_jrk
        t_max_acc = 0.0
        t_max_vel = 0.0
        
        if(pos_diff > min_pos_to_max_vel):
            rospy.logdebug("\n >>> case a1: require const_vel_phase=zero_acc_phase [ /\-----\/ ]" )
            t_max_vel= (pos_diff - min_pos_to_max_vel )/ abs_max_vel
        else:
            rospy.logdebug("\n >>> case a2: calculate Acc corresponds to pos_diff [ /\\/ ]")
            acc = calculate_reached_acc_in_case_no_const_acc_phase(pos_diff, v, abs_max_vel, abs_max_acc, abs_max_jrk)
            t_max_jrk = acc/abs_max_jrk
         
        
    #if  no: check if a const_acc phase is required or not (to satisfy pos_diff) 
    elif(reached_acc_to_max_vel > abs_max_acc):
        rospy.logdebug("case b: maxAcc will be reached !  /'''\\.../")
        min_pos_to_max_vel, t_max_acc = calculate_min_pos_const_acc_time_to_reach_max_acc_and_max_vel(v, abs_max_vel, abs_max_acc, abs_max_jrk)
        reached_vel = abs_max_vel
        reached_acc = abs_max_acc    
        t_max_jrk = abs_max_acc/abs_max_jrk
        t_max_vel = 0.0
        
        if(pos_diff >= min_pos_to_max_vel):
            rospy.logdebug("\n >>> case b1: require const_vel_phase=zero_acc_phase [ /```\------\.../ ]" )
            t_max_vel= (pos_diff - min_pos_to_max_vel )/ abs_max_vel
        else:
            min_pos_to_max_acc = calculate_min_pos_to_reach_max_acc(v, abs_max_vel, abs_max_acc, abs_max_jrk)
            rospy.logdebug("case b2: min_pos_to_max_acc= {}, Dp= {} ".format(min_pos_to_max_acc, pos_diff) )
            if(pos_diff >= min_pos_to_max_acc):
                rospy.logdebug( "\n >>> case b2a: calculate acc_time-reached_vel corresponds to pos_diff [ /````\\..../ ]" )
                acc_time = calculate_const_acc_time(pos_diff, v, abs_max_vel, abs_max_acc, abs_max_jrk)
                t_max_acc = acc_time
            else:
                rospy.logdebug( "\n >>> case b2b: calculate acc corresponds to pos_diff [ /\\/ ]")
                acc = calculate_reached_acc_in_case_no_const_acc_phase(pos_diff, v, abs_max_vel, abs_max_acc, abs_max_jrk)
                t_max_jrk = acc/abs_max_jrk
                t_max_acc = 0.0
                t_max_vel = 0.0
           
    return t_max_jrk, t_max_acc, t_max_vel, reached_vel, reached_acc





### the main function to plan motion profile for a general_velocity-to-general_velocity segment
def traj_segment_planning(p_start, p_end, abs_v_start, abs_v_end, abs_max_vel, abs_max_acc, abs_max_jrk):
    '''
    this function selects a motion profile for a trajectory segment with a given start and end velocities/positions, 
    considering the start and end accelerations/jerks are zeros!        
    '''

    #calculate min_pos required to reach vf from v0   
    abs_min_pos_to_vf, acc_to_vf, t_jrk_to_vf, t_acc_to_vf = calculate_min_pos_reached_acc_jrk_time_acc_time_to_reach_final_vel(abs_v_start, abs_v_end, abs_max_vel, abs_max_acc, abs_max_jrk)
    if abs_min_pos_to_vf > abs(p_end-p_start) and  abs_min_pos_to_vf - abs(p_end-p_start) > 1e-5: # if abs_min_pos_to_vf> abs(p_end-p_start), then these values are not feasible
        print">>> min required position difference to reach v_end from v_start= {} > abs(p_end-p_start)={} ".format(abs_min_pos_to_vf, abs(p_end-p_start) )       
        raise ValueError("non feasible case: violate min_pos_to_vf" )      
        return 0, 0, 0, 0, 0
    
    else: ## then the motion will be: 
        if abs_v_end > abs_v_start: #from v0 to reach vf with dis=minPos_to_vf, then then plan the rest of the required distance (pos_diff - minPos_to_vf) using vf (as vf>v0)
            abs_v=abs_v_end
        else:                       #plan the rest of the required distance (pos_diff - minPos_to_vf) using v0 (as v0>vf), from v0 to reach vf with dis= minPos_to_vf 
            abs_v=abs_v_start
        
        
        if abs( abs(p_end-p_start) - abs_min_pos_to_vf)> 1e-7:
            pos_diff= p_end - p_start - math.copysign( abs_min_pos_to_vf, p_end - p_start)   
            ## plan the rest of the motion using the equal start/end vel case
            t_jrk, t_acc, t_vel, reached_vel, reached_acc = equal_vel_case_planning ( abs(pos_diff), abs_v, abs_max_vel, abs_max_acc, abs_max_jrk)
        else:
            t_jrk=0.0
            t_acc=0.0
            t_vel=0.0
    # return time for both: from v0_to_vf case and for equal_vel_case 
    rospy.logdebug(">>> output of traj_segment_planning: t_jrk_to_vf, t_acc_to_vf, t_jrk, t_acc, t_vel: ")
    rospy.logdebug("{},  {}, {},  {}, {}".format(t_jrk_to_vf, t_acc_to_vf, t_jrk, t_acc, t_vel) ) 
    return t_jrk_to_vf, t_acc_to_vf, t_jrk, t_acc, t_vel













