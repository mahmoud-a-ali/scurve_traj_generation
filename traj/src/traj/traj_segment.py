#!/usr/bin/env python
"""
this file contains main high level planning function "fit_traj_segment" to fit a trajectory segment for given start/end velocities/positions.
fit_traj_segment does high level planning for the segment:
1. check  the given start/end pos/vel values, if they are within the limits or not
2. check the given start/end pos/vel values, if they form feasible/logical case for generic segment or not
3. check motion type (simple +ve/-ve motion or it is complex motion where vel direction will change) 
4. it calls low level planning function "traj_segment_planning" to calculate the values of t_jrk, t_acc, t_vel 
5. it generates pos, vel, acc, jrk vectors using the abovemention times: t_jr, t_acc, t_vel
6. it returns vectors of pos, vel, acc, jrk  
"""
    
from sympy import integrate, Symbol
from sympy.core.numbers import Float

from .piecewise_function import PiecewiseFunction
import traj
import math
import rospy


### Function to assign jerk sign for each phase based on the motion (+ve/-ve): it is determined by start/end vel, and pos_diff 
def assign_jerk_sign_According_to_motion_type(p_start, p_end, v_start, v_end, p_max, v_max, a_max, j_max):
    """
    Function to assign jerk sign for each phase based on the motion (+ve/-ve): it is determined by start/end vel, and pos_diff 
    """    
    abs_v_start = abs(v_start)
    abs_v_end = abs(v_end)   
            
    if v_start == v_end:
         j_max_to_vf=0
         j_max = math.copysign(j_max, (p_end-p_start) ) 
         
    else:# v_end != v_start:
        if v_start*v_end < 0: #won't be need it in complex motion case 
            rospy.logdebug("this a complex motion, stop point will be calculated to join the +ve/-ve motion part " )          
        elif abs_v_start < abs_v_end : #acc motion
            if(v_start >= 0 and v_end >= 0): # positive motion
                j_max_to_vf = j_max #math.copysign(j_max, v_end)
                j_max = math.copysign(j_max, v_end)

            elif (v_start <= 0 and v_end <= 0): # negative motion
                j_max_to_vf = -j_max #math.copysign(j_max, v_end)
                j_max = math.copysign(j_max, v_end)    
                
        else:# v_start > v_end : #dec motion
            if(v_start >= 0 and v_end >= 0): # positive motion
                j_max_to_vf = -j_max #math.copysign(j_max, v_end)
                j_max = math.copysign(j_max, v_end)
            elif (v_start <= 0 and v_end <= 0): # negative motion
                j_max_to_vf = j_max #math.copysign(j_max, v_end)
                j_max = math.copysign(j_max, v_end)
    return j_max_to_vf, j_max







### the main function to fit traj segment with generic start/end velocities 
def fit_traj_segment(p_start, p_end, v_start, v_end, p_max, v_max, a_max, j_max, independent_variable=Symbol('t')):
    """
    main function to fit a trajectory segment for given start/end velocities/positions!
    """
    assert(a_max > 0.0)
    assert(j_max > 0.0)
    assert(v_max > 0.0)
    
    #absolute value of the velocities    
    abs_v_start = abs(v_start)
    abs_v_end = abs(v_end)   
        
        
    ######################## Step_1:  check limits for given start/end velocities/positions ####################
    # if absolute values v_start/v_end/p_end is greater than v_max/p_max, we replace the values with max one
    # another option is to raise error and exit 
    # for p_start: it depends on direction of v_start, as we can not put p_start as p_max if v_start is in +ve direction 
    if(abs(v_start) > v_max):
        rospy.logdebug("\nWarning: \n>>> these values are not feasible:  v_start should be within the limit v_max !")
        v_start = math.copysign(v_max, v_start)

    if(abs(v_end) > v_max):
        rospy.logdebug("\nWarning: \n>>> these values are not feasible,   v_end should be within the limit v_max !")
        v_end = math.copysign(v_max, v_end)

    if( abs(p_end) > p_max):
        rospy.logdebug("\nWarning: \n>>> these values are not feasible,   p_end should be within the limit p_max !")
        p_end = math.copysign(p_max, p_end)
        
    if(abs(p_start) > p_max):
        p_start = math.copysign(p_max, p_start)
        if (p_start*v_start>0.0) or (v_start==0 and p_start*v_end>0.0): #direction of motion 
            print"\nWarning: \n>>> these values are not feasible,  p_start = p_max, and motion in the direction of v_start will violate p_max!"
            raise ValueError("non feasible case: violate p_max" ) 
            
            
    ####### reject unfeasible/iillogical cases 
    if (v_start>0 and v_end>0 and (p_end-p_start)<0): # +ve motion vs -ve pos_diff
        raise ValueError("non feasible case: vel_motion opposite to pos_motion" )
    elif (v_start<0 and v_end<0 and (p_end-p_start)>0): # -ve motion  vs +ve pos_diff
        raise ValueError("non feasible case: vel_motion opposite to pos_motion" )   

        
    ######################## Step_2:  check motion type: complex or simple motion ##########################
    
    #############################################################################
    ## 1) complex motion:  positive and negative velocities, v_start*v_end<0 ####
    #############################################################################
    if (v_start * v_end) < 0 : #complex motion:  positive and negative velocity, check min distance to change diraction of the motion
        minPos_to_zero, acc_to_zero, t_jrk_to_zero, t_acc_to_zero = traj.calculate_minPos_reachAcc_maxJrkTime_maxAccTime_to_final_vel(v_start,   0.0,   v_max, a_max, j_max)
        minPos_to_vf, acc_to_vf, t_jrk_to_vf, t_acc_to_vf         = traj.calculate_minPos_reachAcc_maxJrkTime_maxAccTime_to_final_vel(    0.0, v_end,   v_max, a_max, j_max) 
        pos_diff = p_end - p_start
        pos_dominant = pos_diff - minPos_to_zero - minPos_to_vf
  
        
        ######################## A) complex positive motion case ########################
        if pos_dominant > 0:  ## positive dominant case, main part of the motion is in the +ve direction 
            if v_start < 0 and v_end > 0: # from negative to positive 
                if abs(p_start+minPos_to_zero) > p_max or abs(p_start+minPos_to_zero+minPos_to_vf) > p_max or  abs(p_start+minPos_to_zero+minPos_to_vf+pos_dominant) > p_max:
                    raise ValueError("non feasible case: violate p_max" ) 
                
                rospy.logdebug("\n\n>>>positive dominant case: negative to positive: {}, {}, {}, {}".format(p_start, p_end, v_start, v_end) )
                t_jrk_not_used, t_acc_not_used, t_jrk_dominant, t_acc_dominant, t_vel_dominant = traj.traj_segment_planning(p_start, p_end - minPos_to_zero - minPos_to_vf,       abs_v_end,      abs_v_end,      v_max, a_max, j_max)                           

                segment_jerks_and_durations = [( j_max, t_jrk_to_zero),  (0.0, t_acc_to_zero),  (-j_max, t_jrk_to_zero ),
                                               ( j_max, t_jrk_to_vf),    (0.0, t_acc_to_vf),    (-j_max, t_jrk_to_vf ),
                                               ( j_max, t_jrk_dominant), (0.0, t_acc_dominant), (-j_max, t_jrk_dominant),   (0, t_vel_dominant),(-j_max, t_jrk_dominant), (0.0, t_acc_dominant), (j_max, t_jrk_dominant) ]                    
            elif v_start > 0 and v_end < 0: #from positive to negative
                if abs(p_start+pos_dominant) > p_max or abs(p_start+pos_dominant+minPos_to_zero) > p_max or  abs(p_start+pos_dominant+minPos_to_zero+minPos_to_vf) > p_max:
                    raise ValueError("non feasible case: violate p_max" )               
                
                rospy.logdebug("\n\n>>>positive dominant case: positive to negative: {}, {}, {}, {}".format(p_start, p_end, v_start, v_end))
                t_jrk_not_used, t_acc_not_used, t_jrk_dominant, t_acc_dominant, t_vel_dominant = traj.traj_segment_planning(p_start, p_end-minPos_to_zero-minPos_to_vf, abs_v_start, abs_v_start, v_max, a_max, j_max)                               
               
                segment_jerks_and_durations = [( j_max, t_jrk_dominant), (0.0, t_acc_dominant), (-j_max, t_jrk_dominant),  (0, t_vel_dominant), (-j_max, t_jrk_dominant), (0.0, t_acc_dominant), (j_max, t_jrk_dominant),
                                               (-j_max, t_jrk_to_zero),  (0.0, t_acc_to_zero),  ( j_max, t_jrk_to_zero ),
                                               (-j_max, t_jrk_to_vf),  (0.0, t_acc_to_vf),  (j_max, t_jrk_to_vf ) ]
            else:
                raise ValueError("\n>> should be simple motion instead of complex motion case!" ) 
                
        ######################## B) complex negative motion case ########################
        if pos_dominant < 0:  ## negative dominant case, main part of the motion is in the -ve direction  
            if v_start < 0 and v_end > 0: # from negative to positive
                if abs(p_start+pos_dominant) > p_max or abs(p_start+pos_dominant+minPos_to_zero) > p_max or  abs(p_start+pos_dominant+minPos_to_zero+minPos_to_vf) > p_max:
                    raise ValueError("non feasible case: violate p_max" )                
                
                rospy.logdebug("\n\n>>>negative dominant case: negative to positive: {}, {}, {}, {}".format(p_start, p_end, v_start, v_end))
                t_jrk_not_used, t_acc_not_used, t_jrk_dominant, t_acc_dominant, t_vel_dominant = traj.traj_segment_planning(p_start, p_end-minPos_to_zero-minPos_to_vf, abs_v_start, abs_v_start, v_max, a_max, j_max)                                          
                segment_jerks_and_durations = [(-j_max, t_jrk_dominant), (0.0, t_acc_dominant), ( j_max, t_jrk_dominant),  (0, t_vel_dominant),(j_max, t_jrk_dominant), (0.0, t_acc_dominant), (-j_max, t_jrk_dominant),
                                               ( j_max, t_jrk_to_zero),  (0.0, t_acc_to_zero),  (-j_max, t_jrk_to_zero ),
                                               ( j_max, t_jrk_to_vf),    (0.0, t_acc_to_vf),    (-j_max, t_jrk_to_vf ) ]
            elif v_start > 0 and v_end < 0: #from positive to negative
                if abs(p_start+minPos_to_zero) > p_max or abs(p_start+minPos_to_zero+minPos_to_vf) > p_max or  abs(p_start+minPos_to_zero+minPos_to_vf+pos_dominant) > p_max:
                    raise ValueError("non feasible case: violate p_max" )      
                    
                rospy.logdebug("\n\n>>>negative dominant case: positive to negative: {}, {}, {}, {}".format(p_start, p_end, v_start, v_end)  )         
                t_jrk_not_used, t_acc_not_used, t_jrk_dominant, t_acc_dominant, t_vel_dominant = traj.traj_segment_planning(p_start+ minPos_to_zero + minPos_to_vf, p_end , abs_v_end, abs_v_end,  v_max, a_max, j_max)
          
                segment_jerks_and_durations = [(-j_max, t_jrk_to_zero),  (0.0, t_acc_to_zero),  ( j_max, t_jrk_to_zero ),
                                               (-j_max, t_jrk_to_vf),    (0.0, t_acc_to_vf),    ( j_max, t_jrk_to_vf ),
                                               (-j_max, t_jrk_dominant), (0.0, t_acc_dominant), ( j_max, t_jrk_dominant),  (0, t_vel_dominant), ( j_max, t_jrk_dominant), (0.0, t_acc_dominant), (-j_max, t_jrk_dominant) ]
            else:
                raise ValueError("\n>> should be simple motion instead of complex motion case!" ) 

        ###### check if final_velocity value gives optimal motion to change from +ve/-ve to -ve/+ve
        ## this part can be used later to assign velocity vf in the parameterizarion part
        minPos_v02vf = minPos_to_zero + minPos_to_vf
        if v_start < 0 and v_end > 0: #from -ve to +ve
            if pos_diff < minPos_v02vf:
                rospy.logdebug(">>>>>> non optimal case <<<<<<< ")
        else:
            if pos_diff > minPos_v02vf:
                rospy.logdebug(">>>>>> non optimal case <<<<<<< ")
                

 
    ####################################################################################    
    ##### 2)simple motion:  positive or negative velocity, v0 and vf have same sign ####
    ####################################################################################
    else:
        ## same action will be performed in both simple +ve or simple -ve motion, this part can be used later 
        ############ A) simple positive motion
        if(v_start >= 0 and v_end >= 0): # case one: both are positive
            rospy.logdebug( "\n\n>>>simple postive motion: {}, {}, {}, {} ".format(p_start, p_end, v_start, v_end))

        ############ B) simple negative motion                        
        elif (v_start <= 0 and v_end <= 0): # case two: both are negative
            rospy.logdebug( "\n\n>>>simple negative motion: {}, {}, {}, {} ".format(p_start, p_end, v_start, v_end))
 
        t_jrk_to_vf, t_acc_to_vf, t_jrk, t_acc, t_vel = traj.traj_segment_planning(p_start, p_end, abs_v_start, abs_v_end, v_max, a_max, j_max)
        j_max_to_vf, j_max = assign_jerk_sign_According_to_motion_type(p_start, p_end, v_start, v_end, p_max, v_max, a_max, j_max)
        if abs_v_end > abs_v_start:
            segment_jerks_and_durations = [(j_max_to_vf, t_jrk_to_vf), (0.0, t_acc_to_vf), (-j_max_to_vf, t_jrk_to_vf),    (j_max, t_jrk), (0.0, t_acc), (-j_max, t_jrk), (0.0, t_vel), (-j_max,t_jrk), (0.0, t_acc), (j_max, t_jrk)]
        else:
            segment_jerks_and_durations = [(j_max, t_jrk), (0.0, t_acc), (-j_max, t_jrk), (0.0, t_vel), (-j_max,t_jrk), (0.0, t_acc), (j_max, t_jrk),    (j_max_to_vf, t_jrk_to_vf), (0.0, t_acc_to_vf), (-j_max_to_vf, t_jrk_to_vf)]
    
   
    ### one option to retun segment_jerks_and_durations and send it to JTC and then use it for interpolation on the JTC side
    #return segment_jerks_and_durations
   
   
   ######################## Step_3:  generate pos, vel, acc, jrk using the calculated "segment_jerks_and_durations" ##########################         
 
    p0 = p_start
    v0 = v_start
    a0 = 0.0
    times = [0.0]
    jerk_functions = []
    acceleration_functions = []
    velocity_functions = []
    position_functions = []
    # Integrate jerk starting from the start of the trajectory and going all the way through the end.
    for j0, T in segment_jerks_and_durations:
        times.append(times[-1] + T)
        j = Float(j0)
        a = integrate(j, independent_variable) + a0
        v = integrate(a, independent_variable) + v0
        p = integrate(v, independent_variable) + p0
        jerk_functions.append(j)
        acceleration_functions.append(a)
        velocity_functions.append(v)
        position_functions.append(p)
        a0 = a.subs({independent_variable: T})
        v0 = v.subs({independent_variable: T})
        p0 = p.subs({independent_variable: T})
    position = PiecewiseFunction(times, position_functions, independent_variable)
    velocity = PiecewiseFunction(times, velocity_functions, independent_variable)
    acceleration = PiecewiseFunction(times, acceleration_functions, independent_variable)
    jerk = PiecewiseFunction(times, jerk_functions, independent_variable)
    return position, velocity, acceleration, jerk
