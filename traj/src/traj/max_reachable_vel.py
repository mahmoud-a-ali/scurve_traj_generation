#!/usr/bin/env python
"""
this file contains main low level function to calculate the maximum reachable velocity at the end of the segment 
based on the position difference between p_start and p_end
"""
import rospy
import math
import traj
    
    
### calculate minPos to reach absMaxVel starting with initial_vel
def calculate_minPos_reachAcc_maxJrkTime_maxAccTime_to_absMaxVel_3ph(abs_v_start, vm, am, jm):
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
    a1 =  jm*t +  a0;
    a2 =          a1;

    v1 =  jm*t*t/2.0 +  a0*t   + v0;
    v2 =                a1*ta  + v1;

    p1 =  jm*t*t*t/6.0 +  a0*t*t/2.0    + v0*t  + p0;
    p2 =                  a1*ta*ta/2.0  + v1*ta + p1;
    p3 = -jm*t*t*t/6.0 +  a2*t*t/2.0    + v2*t  + p2;
  
    
    if abs(a2)<=abs(am):
        acc_to_absMaxVel = a2 
        minPos_to_absMaxVel = p3 
    else:
        tj =abs(am/jm) 
        ta= ( abs(vm-v0) - abs((am**2/jm)) )/am        
        
        t = tj
        a1 =  jm*t +  a0;
        a2 =          a1;

        v1 =  jm*t*t/2.0 +  a0*t   + v0;
        v2 =                a1*ta  + v1;

        p1 =  jm*t*t*t/6.0 +  a0*t*t/2.0    + v0*t  + p0;
        p2 =                  a1*ta*ta/2.0  + v1*ta + p1;
        p3 = -jm*t*t*t/6.0 +  a2*t*t/2.0    + v2*t  + p2;  
        
        acc_to_absMaxVel = a2 
        minPos_to_absMaxVel = p3 
    return minPos_to_absMaxVel, acc_to_absMaxVel, tj, ta
         



### calculate minPos to reach absMaxAcc starting with initial_vel
def calculate_minPos_to_absMaxAcc_3ph(v0, vm, am, jm):
    p0=0.0
    a0=0.0
    tj= am/jm    
    t=tj
    ta= 0.0

    a1 =  jm*t +  a0;
    a2 =          a1;

    v1 =  jm*t*t/2.0 +  a0*t   + v0;
    v2 =                a1*ta  + v1;

    p1 =  jm*t*t*t/6.0 +  a0*t*t/2.0    + v0*t  + p0;
    p2 =                  a1*ta*ta/2.0  + v1*ta + p1;
    p3 = -jm*t*t*t/6.0 +  a2*t*t/2.0    + v2*t  + p2;
  
    minPos_to_absMaxAcc = p3
    return minPos_to_absMaxAcc
         
               
         
         
         
         
         

## calculate the time of the const_acc_phase to reach required pos_diff, in case that the absMaxVel won't be reached
def calculate_ta_vend_for_pos_diff_3ph(Dp, abs_v_start, vm, am, jm):
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

    v1 =  jm*t*t/2.0 +  a0*t   + v0;
    v2 =                a1*ta  + v1;
    v3 = -jm*t*t/2.0 +  a2*t   + v2;

  
    absMaxAccTime = ta
    v_end = v3
    return absMaxAccTime, v_end
    
    
    
    
def calculate_tj_reachableAcc_vend_for_pos_diff_3ph(Dp, abs_v_start, vm, am, jm):
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
    
    a1 =  jm*t +  a0;
    a2 =          a1;
    
    v1 =  jm*t*t/2.0 +  a0*t   + v0;
    v2 =                a1*ta  + v1;
    v3 = -jm*t*t/2.0 +  a2*t   + v2;

    
    reached_Acc = a2
    v_end = v3 
    return tj, reached_Acc, v_end
    
    
    
    
    
    
    
    
    
    
    
    
  
         
def max_reachable_vel(abs_pos_diff, abs_v_start, abs_max_pos, abs_max_vel, abs_max_acc, abs_max_jrk):
    """
    the maximum reachable velocity at the end of the segment, based on the position difference: 1. (p_end - p_start), 2.v_start
    this functions return the phase times: tj, ta, tv considering max three phases: [acc profile be like /`````\........ ] 
    a) tj: time to reach max  acc 
    b) ta: time to reach max vel
    c) tv: time with max vel to complete the required position difference p_end-p_start   
    """         
    rospy.logdebug( "\n max_vel_info: pos_diff={}, v_start ={} ".format( abs_pos_diff, abs_v_start) )

    #########################################################################################
    ############ A) if (pos_diff is zero), then time is zero and v_end = v_start ############
    #########################################################################################
    if abs_pos_diff == 0.0:
        rospy.logdebug("\n>>> case A")
        tj= 0.0
        ta= 0.0
        tv= 0.0
        abs_v_end= abs_v_start
        return tj, ta, tv, abs_v_end
        
        
        
    #################################################################################################################
    ############ B) if (pos_diff and v0 have same sign), then (vf is +ve) and (vf is based on pos_diff)  ############
    #################################################################################################################
    elif abs_pos_diff > 0.0 and abs_v_start >= 0.0:  
        rospy.logdebug(  "\n>>> case B")   
        #calculate minPos_to_absMaxVel
        minPos_to_absMaxVel, acc_to_absMaxVel, tj, ta = calculate_minPos_reachAcc_maxJrkTime_maxAccTime_to_absMaxVel_3ph(abs_v_start, abs_max_vel, abs_max_acc, abs_max_jrk)
        rospy.logdebug( "minPos_to_absMaxVel= {}, acc_to_absMaxVel={}, tj={}, ta={}".format(minPos_to_absMaxVel, acc_to_absMaxVel, tj, ta) )
        
        ###### B1) if abs_pos_diff >= minPos_to_absMaxVel, then reach to vm and then continue with vm
        if abs_pos_diff >= minPos_to_absMaxVel:
            rospy.logdebug( "\n>>> case B1")
            abs_v_end = abs_max_vel
            tv = (abs_pos_diff - minPos_to_absMaxVel) / abs_max_vel
            return tj, ta, tv, abs_v_end
        
        
        ###### B2) else abs_pos_diff < minPos_to_absMaxVel, and acc_to_absMaxVel >= abs_max_acc
        else:
            rospy.logdebug( "\n>>> case B2")
            tv= 0.0
            #calculate minPos_to_absMaxAcc
            minPos_to_absMaxAcc = calculate_minPos_to_absMaxAcc_3ph(abs_v_start, abs_max_vel, abs_max_acc, abs_max_jrk)
            rospy.logdebug( "minPos_to_absMaxAcc= {}".format(minPos_to_absMaxAcc) )
        
            #### B2a) if abs_pos_diff >= minPos_to_absMaxAcc, then calculate ta such that it gives  abs_pos_diff, tj is already known: tj= am/jm 
            if abs_pos_diff >= minPos_to_absMaxAcc:
                tj= abs_max_acc/abs_max_jrk
                rospy.logdebug( ">>>case B2a" )
                #calculate ta, v_end:
                ta, abs_v_end = calculate_ta_vend_for_pos_diff_3ph(abs_pos_diff, abs_v_start, abs_max_vel, abs_max_acc, abs_max_jrk)
                rospy.logdebug( "tj={},  ta={},  tv={},     v_end= {}".format( tj, ta, tv,  abs_v_end ) )
                return tj, ta, tv, abs_v_end
                
            #### B2b) else abs_pos_diff < minPos_to_absMaxAcc, then calculate tj such that it gives  abs_pos_diff, ta is already known: ta= 0.0  
            else:
                ta= 0.0
                rospy.logdebug( ">>>case B2b" )
                #calculate tj, a, v_end:
                tj, reached_Acc, abs_v_end = calculate_tj_reachableAcc_vend_for_pos_diff_3ph(abs_pos_diff, abs_v_start, abs_max_vel, abs_max_acc, abs_max_jrk)
                rospy.logdebug(  "tj={},  ta={},  tv={},   reached_Acc={},     v_end={}".format( tj, ta, tv, reached_Acc, abs_v_end ) )
                return tj, ta, tv, abs_v_end
            
            
            
    #################################################################################################################################################################        
    ############ C) if (pos_diff and v0 have different sign), then (vf is opposite to v0 sign) and (vf is based on both pos_diff_v0_0 and pos_diff_0_vf) ############
    #################################################################################################################################################################
    elif abs_pos_diff < 0.0 or abs_v_start < 0.0:#(pos_diff > 0.0 and v_start < 0.0) or (pos_diff < 0.0 and v_start > 0.0):
        rospy.logdebug(  "\n>>> complex case: not implemented yet "  )      
        raise ValueError("Case C: in param_max_vel" )
        return 0.0, 0.0, 0.0, 0.0
        
        
        
        
        
        













