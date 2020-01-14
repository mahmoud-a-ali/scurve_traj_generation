#!/usr/bin/env python

import math
import cubic_eq_roots as rt


def calculate_minPos_localMaxAcc_to_absMaxVel(v, vm, am, jm):
#    print "calculate min_position and localMaxAcc (acc to reach absMaxVel; could be less/higher than absMaxAcc) in case of reaching absMaxVel ... "
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
#    a7 =  jm*t +  a6;
    v1 =  jm*t*t/2.0 +  a0*t   + v0;
    v2 =                a1*ta  + v1;
    v3 = -jm*t*t/2.0 +  a2*t   + v2;
    v4 =                a3*tv  + v3;
    v5 = -jm*t*t/2.0 +  a4*t   + v4;
    v6 =                a5*ta  + v5;
#    v7 =  jm*t*t/2.0 +  a6*t   + v6;
    p1 =  jm*t*t*t/6.0 +  a0*t*t/2.0    + v0*t  + p0;
    p2 =                  a1*ta*ta/2.0  + v1*ta + p1;
    p3 = -jm*t*t*t/6.0 +  a2*t*t/2.0    + v2*t  + p2;
    p4 =                  a3*t*t/2.0    + v3*tv + p3;
    p5 = -jm*t*t*t/6.0 +  a4*t*t/2.0    + v4*t  + p4;
    p6 =                  a5*ta*ta/2.0  + v5*ta + p5;
    p7 =  jm*t*t*t/6.0 +  a6*t*t/2.0    + v6*t  + p6;
    
    minPos_to_absMaxVel = p7
    localMaxAcc_to_absMaxVel = ar
    return minPos_to_absMaxVel, localMaxAcc_to_absMaxVel
    



def calculate_Acc_No_constAccPhase(Dp, v, vm, am ,jm):
#    print "calculate required acc to achieve the required position difference ... "    
    p0=0.0
    a0=0.0
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
#    print "ar, tj: "
#    print ar, ar/jm
 
    t= ar/jm
    ta=0.0
    tv=0.0
       
    a1 =  jm*t +  a0;
    a2 =          a1;
    a3 = -jm*t +  a2;
    a4 =          a3;
    a5 = -jm*t +  a4;
    a6 =          a5;
#    a7 =  jm*t +  a6;
    v1 =  jm*t*t/2.0 +  a0*t   + v0;
    v2 =                a1*ta  + v1;
    v3 = -jm*t*t/2.0 +  a2*t   + v2;
    v4 =                a3*tv  + v3;
    v5 = -jm*t*t/2.0 +  a4*t   + v4;
    v6 =                a5*ta  + v5;
#    v7 =  jm*t*t/2.0 +  a6*t   + v6;
    p1 =  jm*t*t*t/6.0 +  a0*t*t/2.0    + v0*t  + p0;
    p2 =                  a1*ta*ta/2.0  + v1*ta + p1;
    p3 = -jm*t*t*t/6.0 +  a2*t*t/2.0    + v2*t  + p2;
    p4 =                  a3*t*t/2.0    + v3*tv + p3;
    p5 = -jm*t*t*t/6.0 +  a4*t*t/2.0    + v4*t  + p4;
    p6 =                  a5*ta*ta/2.0  + v5*ta + p5;
    p7 =  jm*t*t*t/6.0 +  a6*t*t/2.0    + v6*t  + p6;
    
    acc=ar
    pos_diff = p7-p0
#    print "pos_diff= {} ".format(pos_diff)
    return acc
 


def calculate_minPos_constAccTime_to_absMaxAcc_absMaxVel(v, vm, am, jm):
#    print "calculate minPos and constAccTime to reach both the absMaxAcc and the absMaxVel ... "
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
#    a7 =  jm*t +  a6;
    v1 =  jm*t*t/2.0 +  a0*t   + v0;
    v2 =                a1*ta  + v1;
    v3 = -jm*t*t/2.0 +  a2*t   + v2;
    v4 =                a3*tv  + v3;
    v5 = -jm*t*t/2.0 +  a4*t   + v4;
    v6 =                a5*ta  + v5;
#    v7 =  jm*t*t/2.0 +  a6*t   + v6;
    p1 =  jm*t*t*t/6.0 +  a0*t*t/2.0    + v0*t  + p0;
    p2 =                  a1*ta*ta/2.0  + v1*ta + p1;
    p3 = -jm*t*t*t/6.0 +  a2*t*t/2.0    + v2*t  + p2;
    p4 =                  a3*t*t/2.0    + v3*tv + p3;
    p5 = -jm*t*t*t/6.0 +  a4*t*t/2.0    + v4*t  + p4;
    p6 =                  a5*ta*ta/2.0  + v5*ta + p5;
    p7 =  jm*t*t*t/6.0 +  a6*t*t/2.0    + v6*t  + p6;
    
    minPos_to_absMaxVel= p7-p0
    t_max_acc = ta
    return minPos_to_absMaxVel, t_max_acc



def calculate_minPos_to_absMaxAcc(v, vm, am, jm):
#    print "calculate minPos to reach the absMaxAcc ... "
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
#    a7 =  jm*t +  a6;
    v1 =  jm*t*t/2.0 +  a0*t   + v0;
    v2 =                a1*ta  + v1;
    v3 = -jm*t*t/2.0 +  a2*t   + v2;
    v4 =                a3*tv  + v3;
    v5 = -jm*t*t/2.0 +  a4*t   + v4;
    v6 =                a5*ta  + v5;
#    v7 =  jm*t*t/2.0 +  a6*t   + v6;
    p1 =  jm*t*t*t/6.0 +  a0*t*t/2.0    + v0*t  + p0;
    p2 =                  a1*ta*ta/2.0  + v1*ta + p1;
    p3 = -jm*t*t*t/6.0 +  a2*t*t/2.0    + v2*t  + p2;
    p4 =                  a3*t*t/2.0    + v3*tv + p3;
    p5 = -jm*t*t*t/6.0 +  a4*t*t/2.0    + v4*t  + p4;
    p6 =                  a5*ta*ta/2.0  + v5*ta + p5;
    p7 =  jm*t*t*t/6.0 +  a6*t*t/2.0    + v6*t  + p6;
    
    minPos_absMaxAcc= p7-p0
    return minPos_absMaxAcc
    


def calculate_absMaxAccTime(Dp, v, vm, am, jm):
    # here we wont reach max vel but still we reach max acc     
#    print "calculate the time of the const_acc_phase to reach required pos_diff, wont reach to absMaxVel  ... "
    p0=0.0
#    a0=0.0
    v0=v
    #P7 equation is: (2*am^3 + 3*am^2*jm*ta + am*jm^2*ta^2 + 4*v0*am*jm + 2*v0*jm^2*ta + p0*jm^2)/jm^2
    a=0.0
    b= am*jm**2
    c=3*am**2*jm + 2*v0*jm**2
    d=2*am**3 + 4*v0*am*jm + p0*jm**2 - jm**2*Dp  
    rt1, rt2, rt3, n_rts = rt.real_roots_cubic_eq ( a,  b,  c,  d)
#    print "check: rt1, rt2, rt3, n_rts= "
#    print rt1, rt2, rt3, n_rts

    if n_rts==1:
        ta = rt1
    elif n_rts ==2:
        ta = rt.min_positive_root2(rt1, rt2)
    elif n_rts ==3:
        ta = rt.min_positive_root3(rt1, rt2, rt3)    
    absMaxAccTime = ta
    return absMaxAccTime
    
    
    
    
def check_pf_vf(p0, v0, tj, ta, tv, vm, am ,jm ):
#    print "to check final calculated values of p_end & v_end ... "  
    a0=0
    t = tj
       
    a1 =  jm*t +  a0;
    a2 =          a1;
    a3 = -jm*t +  a2;
    a4 =          a3;
    a5 = -jm*t +  a4;
    a6 =          a5;
#    a7 =  jm*t +  a6;
    v1 =  jm*t*t/2.0 +  a0*t   + v0;
    v2 =                a1*ta  + v1;
    v3 = -jm*t*t/2.0 +  a2*t   + v2;
    v4 =                a3*tv  + v3;
    v5 = -jm*t*t/2.0 +  a4*t   + v4;
    v6 =                a5*ta  + v5;
    v7 =  jm*t*t/2.0 +  a6*t   + v6;
    p1 =  jm*t*t*t/6.0 +  a0*t*t/2.0    + v0*t  + p0;
    p2 =                  a1*ta*ta/2.0  + v1*ta + p1;
    p3 = -jm*t*t*t/6.0 +  a2*t*t/2.0    + v2*t  + p2;
    p4 =                  a3*t*t/2.0    + v3*tv + p3;
    p5 = -jm*t*t*t/6.0 +  a4*t*t/2.0    + v4*t  + p4;
    p6 =                  a5*ta*ta/2.0  + v5*ta + p5;
    p7 =  jm*t*t*t/6.0 +  a6*t*t/2.0    + v6*t  + p6;
    
    pf = p7
    vf = v7
    return pf, vf




def calculate_minPos_reachAcc_maxJrkTime_maxAccTime_to_final_vel(v0, vf, vm, am, jm):
#    print "calculate minPos to reach final_vel starting with initial_vel... "
    p0=0.0
    a0=0.0

    ar = math.sqrt( jm*abs(vf-v0) )
    tj= ar/jm
    ta=0.0

    t=tj
    if vf!=v0:
        jm = ( (vf-v0)/abs(vf-v0) ) *jm
    
    a1 =  jm*t +  a0;
    a2 =          a1;
#    a3 = -jm*t +  a2;
    v1 =  jm*t*t/2.0 +  a0*t   + v0;
    v2 =                a1*ta  + v1;
#    v3 = -jm*t*t/2.0 +  a2*t   + v2;
    p1 =  jm*t*t*t/6.0 +  a0*t*t/2.0    + v0*t  + p0;
    p2 =                  a1*ta*ta/2.0  + v1*ta + p1;
    p3 = -jm*t*t*t/6.0 +  a2*t*t/2.0    + v2*t  + p2;
  
    
    if abs(a2)<=abs(am):
        acc_to_vf = a2
        minPos_to_vf = p3
    else:
        tj =abs(am/jm) 
        ta= ( abs(vf-v0) - abs((am**2/jm)) )/am        
        
        t = tj
        a1 =  jm*t +  a0;
        a2 =          a1;
#        a3 = -jm*t +  a2;
        v1 =  jm*t*t/2.0 +  a0*t   + v0;
        v2 =                a1*ta  + v1;
#        v3 = -jm*t*t/2.0 +  a2*t   + v2;   
        p1 =  jm*t*t*t/6.0 +  a0*t*t/2.0    + v0*t  + p0;
        p2 =                  a1*ta*ta/2.0  + v1*ta + p1;
        p3 = -jm*t*t*t/6.0 +  a2*t*t/2.0    + v2*t  + p2;  
        acc_to_vf = a2
        minPos_to_vf = p3
     
    return minPos_to_vf, acc_to_vf, tj, ta
         
    
    
    
#planing: equal velocity case (vf=v0), it returns timeing for each phase: max_jrk, max_acc, max_vel     
def equal_vel_case_planning (pos_diff, v, abs_max_vel, abs_max_acc, abs_max_jrk):
#    print "plan a segment in case of v_start=v_end"
    minPos_to_absMaxVel, localMaxAcc_to_absMaxVel = calculate_minPos_localMaxAcc_to_absMaxVel(v, abs_max_vel, abs_max_acc, abs_max_jrk)
#    print minPos_to_absMaxVel, localMaxAcc_to_absMaxVel
    
    # check if localMaxAcc_to_absMaxVel < abs_max_acc: 
    #if yes: then no const_acc phase, check if a const_vel phase is required or not (to satisfy pos_diff) 
    if(localMaxAcc_to_absMaxVel<= abs_max_acc):
#        print "case a: maxAcc won't be reached !  /\\/ "
        localMaxVel = abs_max_vel
        localMaxAcc = localMaxAcc_to_absMaxVel      
        t_max_jrk = localMaxAcc_to_absMaxVel/abs_max_jrk
        t_max_acc = 0.0
        t_max_vel = 0.0
        
        if(pos_diff > minPos_to_absMaxVel):
            print"\n >>> case a1: require const_vel_phase=zero_acc_phase [ /\-----\/ ]" 
            t_max_vel= (pos_diff - minPos_to_absMaxVel )/ abs_max_vel
        else:
            print "\n >>> case a2: calculate Acc corresponds to pos_diff [ /\\/ ]"
            acc = calculate_Acc_No_constAccPhase(pos_diff, v, abs_max_vel, abs_max_acc, abs_max_jrk)
#            print "case a2: acc= {}".format(acc)
            t_max_jrk = acc/abs_max_jrk
         
        
    #if  no: check if a const_acc phase is required or not (to satisfy pos_diff) 
    elif(localMaxAcc_to_absMaxVel > abs_max_acc):
#        print "case b: maxAcc will be reached !  /'''\\.../"
        minPos_to_absMaxVel, t_max_acc = calculate_minPos_constAccTime_to_absMaxAcc_absMaxVel(v, abs_max_vel, abs_max_acc, abs_max_jrk)
#        print "case b: minPos_to_absMaxVel= {}, t_max_acc= {} ".format(minPos_to_absMaxVel, t_max_acc)
        localMaxVel = abs_max_vel
        localMaxAcc = abs_max_acc    
        t_max_jrk = abs_max_acc/abs_max_jrk
        t_max_vel = 0.0
        
        if(pos_diff >= minPos_to_absMaxVel):
            print "\n >>> case b1: require const_vel_phase=zero_acc_phase [ /```\------\.../ [" 
            t_max_vel= (pos_diff - minPos_to_absMaxVel )/ abs_max_vel
#            print "case b1: t_max_acc= {}, t_max_vel= {} ".format(t_max_acc, t_max_vel)
        else:
#            print "case b2: two possible cases " 
            minPos_absMaxAcc = calculate_minPos_to_absMaxAcc(v, abs_max_vel, abs_max_acc, abs_max_jrk)
#            print "case b2: minPos_absMaxAcc= {}, Dp= {} ".format(minPos_absMaxAcc, pos_diff)
            if(pos_diff >= minPos_absMaxAcc):
                print "\n >>> case b2a: calculate absMaxAccTime-localMaxVel corresponds to pos_diff [ /````\\..../ ]" 
                absMaxAccTime = calculate_absMaxAccTime(pos_diff, v, abs_max_vel, abs_max_acc, abs_max_jrk)
#                print "case b2a: absMaxAccTime= {}".format(absMaxAccTime)
                t_max_acc = absMaxAccTime
            else:
                print "\n >>> case b2b: calculate acc corresponds to pos_diff [ /\\/ ]"
                acc = calculate_Acc_No_constAccPhase(pos_diff, v, abs_max_vel, abs_max_acc, abs_max_jrk)
#                print "case b2b: acc= {}".format(acc)
                t_max_jrk = acc/abs_max_jrk
                t_max_acc = 0.0
                t_max_vel = 0.0
                
    return t_max_jrk, t_max_acc, t_max_vel, localMaxVel, localMaxAcc



### the main function to plan motion profile for a general_velocity-to-general_velocity segment
def traj_segment_planning(p_start, p_end, v_start, v_end, abs_max_vel, abs_max_acc, abs_max_jrk):
    """
    plan motion profile for a traj segment with given start and end velocities/positions, considering the start and end accelerations/jerks are zeros!        
    """    
    #calculate min_pos required to reach vf from v0   
    minPos_to_vf, acc_to_vf, t_jrk_to_vf, t_acc_to_vf = calculate_minPos_reachAcc_maxJrkTime_maxAccTime_to_final_vel(v_start, v_end, abs_max_vel, abs_max_acc, abs_max_jrk)
#    print "minPos_to_vf, acc_to_vf, tj, ta: "
#    print minPos_to_vf, acc_to_vf, t_jrk_to_vf, t_acc_to_vf
    if minPos_to_vf > abs(p_end-p_start):
        print"\nWarning: \n>>> this velocities-positions combination is not feasible considering monotonic motion along the segment !"
        print">>> min required position difference to reach v_end from v_start is {}, and the abs(p_end-p_start) is {} \n".format(minPos_to_vf, abs(p_end-p_start))
        return 0, 0, 0, 0, 0
    else:
        if v_end>v_start:
            v=v_end
        else:
            v=v_start
        
        pos_diff= abs(p_end - p_start) - minPos_to_vf   
        t_jrk, t_acc, t_vel, localMaxVel, localMaxAcc = equal_vel_case_planning (pos_diff, v, abs_max_vel, abs_max_acc, abs_max_jrk)
    return t_jrk_to_vf, t_acc_to_vf, t_jrk, t_acc, t_vel













