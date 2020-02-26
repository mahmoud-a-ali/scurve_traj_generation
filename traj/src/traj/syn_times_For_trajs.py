#!/usr/bin/env python
import math
import traj 
from matplotlib import pyplot as plt
import rospy
import numpy as np
import time

'''
example to parameterize trajectory by calculating the max_reaxchable_vel at the end of each segment starting with v_start to achieve pos_diff= p_end - p_start
currently wotking on +ve velocity only

'''

def calculate_new_Tph(Tph, T_new):
    if sum(Tph) ==0:
        return  Tph       
        #raise ValueError("no motion case" )  
    return [ (T_new-sum(Tph) )*t/sum(Tph) + t for t in Tph    ]
        
    
def path_max_vel(path, v_start, v_end):
    # absolute values 
    abs_v_start = abs(v_start)
    abs_ve_end = abs(v_end)    
    # get po dir at each waypoint and indx for stop point where velocity changes its direction
    wpts_vel_dir, stp_idx= traj.set_stp_pts_to_zero(path)
    
    # convert path to positive path, so we can calculate velocity in one positive direction 
    mono_path = [ abs(pt) for pt in path]
    
    
    ### divide traj to small set of trajs 
    stp_idx = [0] + [id for id in stp_idx] + [ len(path)-1]
    path_set = [ mono_path[i:j+1] for i,j in zip(stp_idx[:-1], stp_idx[1:])]
    #print " len(path_set): {}".format(len(path_set))
    
    
    ## estimate velocity between each two stop points
    Estimated_vel = []
    Estimated_vel.append(abs_v_start)   
    for pth in range (0, len(path_set) ):
        waypts = path_set[pth]
        #print waypts
        n_segs = len(waypts) -1
        v_forward = 0.0
        v_backward= 0.0    
        
        if pth==0:  ##for the first potion of the traj, before the first stop point
            v_forward = abs_v_start
            v_backward = 0.0        
        if pth== len(path_set) -1 : ##for the last potion of the traj, after the last stop point
            v_forward = 0.0
            v_backward = abs_ve_end
            if len(path_set) -1 == 0 : ##if it does not have stop point 
                v_forward = abs_v_start
                v_backward = abs_ve_end
        #two vectors for forward/backward max_reachable_vel
        forward_max_vel = [ v_forward ]
        backward_max_vel = [ v_backward ]
        
        ### estimate max reachable velocity
        for wpt in range(0, n_segs ) :
            tj, ta, tv, v_frwrd = traj.max_reachable_vel( abs(waypts[wpt] - waypts[wpt+1]), v_forward, abs_max_pos, abs_max_vel, abs_max_acc, abs_max_jrk)
            #print "abs_pos_diff={}, v_forward={}, nxt_v={} ".format(abs(waypts[wpt] - waypts[wpt+1]), v_forward, v_frwrd)        
            tj, ta, tv, v_bkwrd = traj.max_reachable_vel( abs(waypts[n_segs-wpt]- waypts[n_segs-wpt-1]), v_backward, abs_max_pos, abs_max_vel, abs_max_acc, abs_max_jrk)
            #print "abs_pos_diff={}, v_forward={}, nxt_v={} ".format(abs(waypts[n_segs-wpt]- waypts[n_segs-wpt-1]), v_backward, v_bkwrd)        
    
            forward_max_vel.append(  v_frwrd )
            backward_max_vel.append( v_bkwrd )
            v_forward = v_frwrd # to be used as v_start of the next segment
            v_backward = v_bkwrd
        
        backward_max_vel.reverse()
        #fig = plt.figure()    
        #plt.plot( forward_max_vel, 'b*')
        #plt.plot( forward_max_vel, 'bo', label='forward_vel')
        #plt.plot( backward_max_vel,  'r*')
        #plt.plot( backward_max_vel,  'ro', label='backwar_vel')
        #plt.xlabel("waypoints")
        #plt.ylabel("velocity")
        #plt.legend()
        #plt.show()
        
    
        ##check condition when v_start is not feasible: v_start > max_v_start calculated using the backward loop 
        if forward_max_vel[0] > backward_max_vel[0]:
            print "v_start should be <= {}".format( backward_max_vel[0] )
            raise ValueError("v_start is not feasible" )     
            
        # calcuate max_rechable_vels that grantee v_end at the end of the trajectory for this portion of traj
        estimated_vel =  [ min(v) for v in zip( forward_max_vel, backward_max_vel)] 
        #accumlate the estimated velocity per each portion till complete estimated vel for all portions/waypoints 
        Estimated_vel = Estimated_vel + [v for v in estimated_vel[1:] ]
    
    ## retrieve the direction at each way point 
    Estimated_vel=[v*dir for v, dir in zip(Estimated_vel, wpts_vel_dir) ]
    return Estimated_vel










##### find max_ velocity per path in a certain direction 
def path_max_vel_per_dir(path, v_init):     
    # get po dir at each waypoint and indx for stop point where velocity changes its direction
    wpts_vel_dir, stp_idx= traj.set_stp_pts_to_zero(path)
    stp_idx =  [id for id in stp_idx] 
    # monotonic: convert path to positive path, so we can calculate velocity in one positive direction 
    waypts = [ abs(pt) for pt in path]
    n_wpts = len(waypts) 
    max_vel_vec = [ abs(v_init)   ]  
    ### estimate max reachable velocity
    for wpt in range(0, n_wpts-1 ) :
        if wpt+1 in stp_idx:
            v_nxt = 0.0
        else:
            tj, ta, tv, v_nxt = traj.max_reachable_vel( abs(waypts[wpt] - waypts[wpt+1]), max_vel_vec[wpt], abs_max_pos, abs_max_vel, abs_max_acc, abs_max_jrk)
            #print "abs_pos_diff={}, v_forward={}, nxt_v={} ".format(abs(waypts[wpt] - waypts[wpt+1]), v_forward, v_frwrd)        
        max_vel_vec.append(  v_nxt )      
    return max_vel_vec
        






        
### find forward and backward vel per path (1dof path)
def find_max_estimated_vel_per_path(path, v_start, v_end):
    Frwd_max_vel = path_max_vel_per_dir(path, v_start)
    Bkwd_max_vel = path_max_vel_per_dir(path[::-1], v_end)  #L[::-1]
    Bkwd_max_vel.reverse()
    
    ##check condition when v_start or v_end is not feasible: v_start > max_v_start calculated using the backward loop or Vs
    if Frwd_max_vel[0] > Bkwd_max_vel[0] or Frwd_max_vel[-1] < Bkwd_max_vel[-1]:
        print "abs_v_start={}, abs_max_v_start={} ".format( Frwd_max_vel[0] , Bkwd_max_vel[0] )
        print "abs_v_end  ={}, abs_max_v_end  ={} ".format( Bkwd_max_vel[-1], Frwd_max_vel[-1]  )
        raise ValueError("v_start is not feasible" )     
        
    ##should have the condition of stp point in each dof seperately, the nxt two lines are fake for now 
    wpts_vel_dir, stp_idx= traj.set_stp_pts_to_zero(path)
    stp_idx =  [id for id in stp_idx] 
        
    # calcuate max_rechable_vels that grantee v_end at the end of the trajectory for this portion of traj
    estimated_vel =  [ min(v) for v in zip( Frwd_max_vel, Bkwd_max_vel)] 
    
    ## retrieve the direction at each way point 
    wpts_vel_dir, stp_idx= traj.set_stp_pts_to_zero(path)
    estimated_vel=[v*dir for v, dir in zip(estimated_vel, wpts_vel_dir) ]
    return Frwd_max_vel, Bkwd_max_vel, estimated_vel






######### find estimated max vel for traj, this is not time synchronized
def find_max_estimated_vel_per_ndof_path(path, v_start, v_end):
    Estimated_vel = []    
    for pth in range(0, len(path) ):
        Frwd_max_vel, Bkwd_max_vel, estimated_vel =  find_max_estimated_vel_per_path(path[pth], v_start[pth], v_end[pth])
        Estimated_vel.append( estimated_vel )
    return Estimated_vel
    
    
    
    

### find time to max vel for all dof per one segment:
def find_time_per_max_vel_per_ndof(Pos_diff, Vel_init):
    if len(Pos_diff) != len(Vel_init):
        raise ValueError("len(Pos_diff)={} != len(Vel_init)={}".format( len(Pos_diff), len(Vel_init) ) )  
    n_jt = len(Pos_diff) 
    Vel_nxt = []
    jrk_sgn_dur   = []
    ### estimate max reachable velocity
    for jt in range(0, n_jt ) :
        tj, ta, tv, v_nxt = traj.max_reachable_vel( abs(Pos_diff[jt]), Vel_init[jt], abs_max_pos, abs_max_vel, abs_max_acc, abs_max_jrk)
        Vel_nxt.append(  v_nxt )  
        jrk_dur = traj.calculate_jerk_sign_and_duration(0,  abs(Pos_diff[jt]), Vel_init[jt], Vel_nxt[jt], abs_max_pos, abs_max_vel, abs_max_acc, abs_max_jrk)        
        jrk_sgn_dur.append( jrk_dur )
    return jrk_sgn_dur, Vel_nxt
        
        
        
        
        
###find jrk for time and pos_diff
def calculate_jrk_for_T_pos_diff( pos_diff, v_init, J_jt_ph, T_jt_ph):
    pd = pos_diff
    v0 = v_init
    
    T =  T_jt_ph 
#    [f(x) for x in sequence if condition]
#    [f(x) if condition else g(x) for x in sequence]
    s0 = 0.0 if J_jt_ph[0]==0.0 else math.copysign(1, J_jt_ph[0] ) 
    s1 = 0.0 if J_jt_ph[1]==0.0 else math.copysign(1,  J_jt_ph[1] ) 
    s2 = 0.0 if J_jt_ph[2]==0.0 else math.copysign(1,  J_jt_ph[2] ) 
    s3 = 0.0 if J_jt_ph[3]==0.0 else math.copysign(1,  J_jt_ph[3] ) 
    s4 = 0.0 if J_jt_ph[4]==0.0 else math.copysign(1,  J_jt_ph[4] ) 
    s5 = 0.0 if J_jt_ph[5]==0.0 else math.copysign(1,  J_jt_ph[5] ) 
    s6 = 0.0 if J_jt_ph[6]==0.0 else math.copysign(1,  J_jt_ph[6] ) 
    s7 = 0.0 if J_jt_ph[7]==0.0 else math.copysign(1,  J_jt_ph[7] ) 
    s8 = 0.0 if J_jt_ph[8]==0.0 else math.copysign(1,  J_jt_ph[8] ) 
    s9 = 0.0 if J_jt_ph[9]==0.0 else math.copysign(1,  J_jt_ph[9] ) 
    sgn=[ s0, s1, s2, s3, s4, s5, s6, s7, s8, s9 ]
#    print s0, s1, s2, s3, s4, s5, s6, s7, s8, s9
    t0=T[0] 
    t1=T[1] 
    t2=T[2]
    t3=T[3] 
    t4=T[4]
    t5=T[5]
    t6=T[6]
    t7=T[7]
    t8=T[8]
    t9=T[9]

    jr = -(t0*v0 - pd + t1*v0 + t2*v0 + t3*v0 + t4*v0 + t5*v0 + t6*v0 + t7*v0 + t8*v0 + t9*v0)/((s0*t0**3)/6 + (s0*t0**2*t1)/2 + (s0*t0**2*t2)/2 +
    (s0*t0**2*t3)/2 + (s0*t0**2*t4)/2 + (s0*t0**2*t5)/2 + (s0*t0**2*t6)/2 + (s0*t0**2*t7)/2 + (s0*t0**2*t8)/2 + (s0*t0**2*t9)/2 + (s0*t0*t1**2)/2 + s0*t0*t1*t2 + 
    s0*t0*t1*t3 + s0*t0*t1*t4 + s0*t0*t1*t5 + s0*t0*t1*t6 + s0*t0*t1*t7 + s0*t0*t1*t8 + s0*t0*t1*t9 + (s0*t0*t2**2)/2 + s0*t0*t2*t3 + s0*t0*t2*t4 + 
    s0*t0*t2*t5 + s0*t0*t2*t6 + s0*t0*t2*t7 + s0*t0*t2*t8 + s0*t0*t2*t9 + (s0*t0*t3**2)/2 + s0*t0*t3*t4 + s0*t0*t3*t5 + s0*t0*t3*t6 + s0*t0*t3*t7 + 
    s0*t0*t3*t8 + s0*t0*t3*t9 + (s0*t0*t4**2)/2 + s0*t0*t4*t5 + s0*t0*t4*t6 + s0*t0*t4*t7 + s0*t0*t4*t8 + s0*t0*t4*t9 + (s0*t0*t5**2)/2 + s0*t0*t5*t6 + 
    s0*t0*t5*t7 + s0*t0*t5*t8 + s0*t0*t5*t9 + (s0*t0*t6**2)/2 + s0*t0*t6*t7 + s0*t0*t6*t8 + s0*t0*t6*t9 + (s0*t0*t7**2)/2 + s0*t0*t7*t8 + s0*t0*t7*t9 +
    (s0*t0*t8**2)/2 + s0*t0*t8*t9 + (s0*t0*t9**2)/2 + (s1*t1**3)/6 + (s1*t1**2*t2)/2 + (s1*t1**2*t3)/2 + (s1*t1**2*t4)/2 + (s1*t1**2*t5)/2 + (s1*t1**2*t6)/2 +
    (s1*t1**2*t7)/2 + (s1*t1**2*t8)/2 + (s1*t1**2*t9)/2 + (s1*t1*t2**2)/2 + s1*t1*t2*t3 + s1*t1*t2*t4 + s1*t1*t2*t5 + s1*t1*t2*t6 + s1*t1*t2*t7 + s1*t1*t2*t8 +
    s1*t1*t2*t9 + (s1*t1*t3**2)/2 + s1*t1*t3*t4 + s1*t1*t3*t5 + s1*t1*t3*t6 + s1*t1*t3*t7 + s1*t1*t3*t8 + s1*t1*t3*t9 + (s1*t1*t4**2)/2 + s1*t1*t4*t5 +
    s1*t1*t4*t6 + s1*t1*t4*t7 + s1*t1*t4*t8 + s1*t1*t4*t9 + (s1*t1*t5**2)/2 + s1*t1*t5*t6 + s1*t1*t5*t7 + s1*t1*t5*t8 + s1*t1*t5*t9 + (s1*t1*t6**2)/2 + 
    s1*t1*t6*t7 + s1*t1*t6*t8 + s1*t1*t6*t9 + (s1*t1*t7**2)/2 + s1*t1*t7*t8 + s1*t1*t7*t9 + (s1*t1*t8**2)/2 + s1*t1*t8*t9 + (s1*t1*t9**2)/2 + (s2*t2**3)/6 + 
    (s2*t2**2*t3)/2 + (s2*t2**2*t4)/2 + (s2*t2**2*t5)/2 + (s2*t2**2*t6)/2 + (s2*t2**2*t7)/2 + (s2*t2**2*t8)/2 + (s2*t2**2*t9)/2 + (s2*t2*t3**2)/2 + s2*t2*t3*t4 + 
    s2*t2*t3*t5 + s2*t2*t3*t6 + s2*t2*t3*t7 + s2*t2*t3*t8 + s2*t2*t3*t9 + (s2*t2*t4**2)/2 + s2*t2*t4*t5 + s2*t2*t4*t6 + s2*t2*t4*t7 + s2*t2*t4*t8 + s2*t2*t4*t9 +
    (s2*t2*t5**2)/2 + s2*t2*t5*t6 + s2*t2*t5*t7 + s2*t2*t5*t8 + s2*t2*t5*t9 + (s2*t2*t6**2)/2 + s2*t2*t6*t7 + s2*t2*t6*t8 + s2*t2*t6*t9 + (s2*t2*t7**2)/2 + 
    s2*t2*t7*t8 + s2*t2*t7*t9 + (s2*t2*t8**2)/2 + s2*t2*t8*t9 + (s2*t2*t9**2)/2 + (s3*t3**3)/6 + (s3*t3**2*t4)/2 + (s3*t3**2*t5)/2 + (s3*t3**2*t6)/2 + (s3*t3**2*t7)/2 +
    (s3*t3**2*t8)/2 + (s3*t3**2*t9)/2 + (s3*t3*t4**2)/2 + s3*t3*t4*t5 + s3*t3*t4*t6 + s3*t3*t4*t7 + s3*t3*t4*t8 + s3*t3*t4*t9 + (s3*t3*t5**2)/2 + s3*t3*t5*t6 +
    s3*t3*t5*t7 + s3*t3*t5*t8 + s3*t3*t5*t9 + (s3*t3*t6**2)/2 + s3*t3*t6*t7 + s3*t3*t6*t8 + s3*t3*t6*t9 + (s3*t3*t7**2)/2 + s3*t3*t7*t8 + s3*t3*t7*t9 + (s3*t3*t8**2)/2 +
    s3*t3*t8*t9 + (s3*t3*t9**2)/2 + (s4*t4**3)/6 + (s4*t4**2*t5)/2 + (s4*t4**2*t6)/2 + (s4*t4**2*t7)/2 + (s4*t4**2*t8)/2 + (s4*t4**2*t9)/2 + (s4*t4*t5**2)/2 + s4*t4*t5*t6 +
    s4*t4*t5*t7 + s4*t4*t5*t8 + s4*t4*t5*t9 + (s4*t4*t6**2)/2 + s4*t4*t6*t7 + s4*t4*t6*t8 + s4*t4*t6*t9 + (s4*t4*t7**2)/2 + s4*t4*t7*t8 + s4*t4*t7*t9 + (s4*t4*t8**2)/2 +
    s4*t4*t8*t9 + (s4*t4*t9**2)/2 + (s5*t5**3)/6 + (s5*t5**2*t6)/2 + (s5*t5**2*t7)/2 + (s5*t5**2*t8)/2 + (s5*t5**2*t9)/2 + (s5*t5*t6**2)/2 + s5*t5*t6*t7 + s5*t5*t6*t8 + 
    s5*t5*t6*t9 + (s5*t5*t7**2)/2 + s5*t5*t7*t8 + s5*t5*t7*t9 + (s5*t5*t8**2)/2 + s5*t5*t8*t9 + (s5*t5*t9**2)/2 + (s6*t6**3)/6 + (s6*t6**2*t7)/2 + (s6*t6**2*t8)/2 + 
    (s6*t6**2*t9)/2 + (s6*t6*t7**2)/2 + s6*t6*t7*t8 + s6*t6*t7*t9 + (s6*t6*t8**2)/2 + s6*t6*t8*t9 + (s6*t6*t9**2)/2 + (s7*t7**3)/6 + (s7*t7**2*t8)/2 + (s7*t7**2*t9)/2 +
    (s7*t7*t8**2)/2 + s7*t7*t8*t9 + (s7*t7*t9**2)/2 + (s8*t8**3)/6 + (s8*t8**2*t9)/2 + (s8*t8*t9**2)/2 + (s9*t9**3)/6)
 
    J_ph = [jr*s for s in sgn]
    return J_ph







#calculate pos,vel,acc at each phase change (inflection points): 
def calculate_pos_vel_Acc_jrk_ipts(J, T, v0):
    A=[0.0]
    V=[ v0]
    P=[0.0]
    for i in range (0, 10):
        A.append( J[i]*T[i]        +  A[i] )
        V.append( J[i]*T[i]**2/2.0 +  A[i]*T[i]          + V[i]  )
        P.append( J[i]*T[i]**3/6.0 +  A[i]*T[i]**2/2.0   + V[i]*T[i]  + P[i] )
    return P[-1], V[-1]







## fund to calculate forward/reverse time_synchronized vel per ndof 
def find_estimated_vel_dur_jrk_for_direction_ndof(path, v_start, v_end):
    
    n_jts  = len(path)
    n_wpts = len(path[0])
    n_segs = n_wpts - 1    
    
    np_path = np.array(path)
    Pos_diff = np.diff(np_path)
    
    T_jt_seg = []
    J_jt_seg = []
    V_jt_seg = []
    V_jt_seg.append(v_start)


    for seg in range(0, n_segs):
        pos_diff = Pos_diff[:,seg] #per seg for all jts
        v_init = V_jt_seg[seg]
        
        jrk_sgn_dur, Vel_nxt_useless_unsyn = find_time_per_max_vel_per_ndof( abs(pos_diff), v_init )
        #convert duration and jrk to 1d list
        jrk_dur_arr = np.array( jrk_sgn_dur) #per seg for all jts
        T_jt_ph =  jrk_dur_arr[:,:,1].tolist()
        J_jt_ph =  jrk_dur_arr[:,:,0].tolist()
    
        
        ## find max time 
        T_jt = [sum(T_ph) for T_ph in T_jt_ph]
        max_t = max(T_jt)
      
    
        ## take max time and calculate max vel according to that time, satisfy pos_diff using different value of jrk 
        for jt in range(0, n_jts):
            T_jt_ph[jt] = calculate_new_Tph(T_jt_ph[jt], max_t)#new_syn_T_jt_ph
            print "new_syn_T_jt_ph: {}".format(T_jt_ph[jt])
            #what to do in case of zeros
    
        V_nxt = []
        ## calculate new jrk value
        for jt  in range(0, n_jts):
            if sum(T_jt_ph[jt]) == 0.0 :
                J_jt_ph[jt] = T_jt_ph[jt]
            else:
                J_jt_ph[jt] = calculate_jrk_for_T_pos_diff( abs( pos_diff[jt] ),  v_init[jt],  J_jt_ph[jt],  T_jt_ph[jt])#new_syn_T_jt_ph
            print "new_syn_J_jt_ph: {}".format(J_jt_ph[jt])  
            #print "print J_jt_ph[jt][0]: {}".format(J_jt_ph[jt][0])        
            if abs(J_jt_ph[jt][0]) > 10.1:
                raise ValueError(" syn_jrk ({}) violates limit ".format(J_jt_ph[jt][0]) )  
            dp, vnxt = calculate_pos_vel_Acc_jrk_ipts(J_jt_ph[jt], T_jt_ph[jt], v_init[jt])
            V_nxt.append(vnxt) 
            print dp, vnxt 
    
    
        T_jt_seg.append( T_jt_ph )  
        J_jt_seg.append( J_jt_ph )  
        V_jt_seg.append(V_nxt)
        
    V_jt_seg = np.array( V_jt_seg)
    V_jt_seg = np.rot90(V_jt_seg).tolist()
    V_jt_seg.reverse()  
    
    T_jt_seg = np.array( T_jt_seg)
    T_jt_seg = np.rot90(T_jt_seg).tolist()
    T_jt_seg.reverse() 
    
    J_jt_seg = np.array( J_jt_seg)
    J_jt_seg = np.rot90(J_jt_seg).tolist()
    J_jt_seg.reverse() 

    
    return T_jt_seg, J_jt_seg, V_jt_seg








   
###################### start node, and set log_level to degub ######################
#set log_level to debug
rospy.init_node('traj_segment_node')#, log_level=rospy.DEBUG)
t_init = time.time()
##### check 
abs_max_pos = 30.0
abs_max_vel = 3.0
abs_max_acc = 4.0
abs_max_jrk = 10.0



#trajectory waypoints, different traj
path = [ [0.0, 0.2, 0.8, 2.0, 2.8, 1.3, 2.2, 3.50, 2.8, 3.5], 
         [0.5, 0.6, 3.2, 1.5, 0.9, 0.3, 0.8, 1.00, 2.0, 0.5] ]


path = [ [1.0, 2.0, 3.0, 2.0,  3.0, 4.0, 2.0, 1.0, 2.0,  3.0], 
         [0.2, 0.5, 1.0, 1.2,  2.5, 2.8, 3.0, 4.2, 4.4,  5.0]  ]


positions= [] 
positions.append( [0.001012041720806261, -0.0041338839998457414, -0.3364954637375829, 3.1335235418029623, -1.9007831579863803, -0.0016187068912901527]  )
positions.append( [0.0009791899982615775, -0.004000142678524483, -0.336361429598184, 3.1335236977034997, -1.900782568953615, -0.0016515062354158705]    )
positions.append( [-0.010225007491410358, 0.041612813371950455, -0.2906486068948853, 3.133576868139441, -1.9005816772219828, -0.01283783987793361]      )
positions.append( [-0.02146205670362698, 0.08735951074374665, -0.24480175005218777, 3.1336301944759195, -1.9003801964575855, -0.02405697286457707]      )
positions.append( [-0.032699105915843595, 0.13310620811554283, -0.19895489320949022, 3.133683520812398, -1.900178715693188, -0.03527610585122053]       )
positions.append( [-0.04393615512806022, 0.17885290548733904, -0.15310803636679265, 3.1337368471488767, -1.8999772349287907, -0.04649523883786398]      )
positions.append(  [-0.05517320434027684, 0.22459960285913524, -0.10726117952409508, 3.1337901734853553, -1.8997757541643931, -0.05771437182450745]     )
positions.append(  [-0.06641025355249344, 0.27034630023093137, -0.06141432268139757, 3.133843499821834, -1.8995742733999958, -0.0689335048111509]       )
positions.append( [-0.07764730276471007, 0.31609299760272763, -0.015567465838699945, 3.1338968261583124, -1.8993727926355983, -0.08015263779779436]     )
positions.append(  [-0.08888435197692669, 0.3618396949745238, 0.03027939100399757, 3.133950152494791, -1.899171311871201, -0.09137177078443781]         )
positions.append( [-0.10008854946659862, 0.4074526510249987, 0.07599221370729628, 3.1340033229307322, -1.8989704201395687, -0.10255810442695557]        )
positions.append( [-0.10012140118914331, 0.40758639234632, 0.07612624784669514, 3.1340034788312696, -1.8989698311068035, -0.10259090377108128]          )
#positions.append( [0.0 for pt in range (0, 6) ]          )


### Jon's path
#positions =[]
#positions.append( [ 0.0, 0.0, 0.0, 0.0 ] )
#positions.append( [ 1.5, 0.7, 0.3, 0.0 ] )
#positions.append( [ 0.0, 0.0, 0.0, 0.0 ] )
#positions.append( [-1.5, 0.7, 0.3, 0.0 ] )
#positions.append( [ 0.0, 0.0, 0.0, 0.0 ] )
#positions.append( [ 0.0, 0.0, 0.0 ] )
#positions.append( [ 1.5, 0.7, 0.3 ] )
#positions.append( [ 0.0, 0.0, 0.0 ] )
#positions.append( [1.5, 0.7, 0.3 ] )
#positions.append( [ 0.0, 0.0, 0.0 ] )
#

path_np = np.array( positions )
path = np.rot90( path_np ).tolist()
path.reverse()

positions_bk = positions[::-1] ## make it goes backward
bk_path_np = np.array( positions_bk )
bk_path = np.rot90( bk_path_np ).tolist()
bk_path.reverse()

#print path[0]
#exit()

#path = [ [.10, .20, .30, .40,  .50, 0.3, 0.1], 
#         [0.2, 0.4, 0.6, 0.8,  1.0, 0.6, 0.2]  ]

#path = [ [0.0, 1.5, 0.0, -1.50,  0.0], 
#         [0.0, 0.7, 0.0,  0.70,  0.0],
#         [0.0, 0.3, 0.0,  0.3,  0.0]  ]
# 

path = [ (1.0, 2.0, 3.5), (1.0, 2.0, 3.8)  ] 
bk_path = [ pth[::-1] for pth in path]


n_jts  = len(path)
n_wpts = len(path[0])
n_segs = n_wpts - 1

#start/end velocity of the trajectory
v_start = [0.0*jt for jt in range(0, n_jts)]
v_end   = [0.0 for jt in range(0, n_jts)]

print path
print bk_path
print n_jts, n_wpts, n_segs
############################################ Second approach: find max reachable vel per individual seg for all paths #######################


fw_T_jt_seg, fw_J_jt_seg, fw_V_jt_seg = find_estimated_vel_dur_jrk_for_direction_ndof(path, v_start, v_end)
bk_T_jt_seg, bk_J_jt_seg, bk_V_jt_seg = find_estimated_vel_dur_jrk_for_direction_ndof(bk_path, v_end, v_start)

fig = plt.figure()   

for jt in range(0, n_jts):
    #print "fw_V_jt_seg[jt]: {}".format( fw_V_jt_seg[jt] )
    plt.plot( fw_V_jt_seg[jt] ,label='fw_vel_jt_{}'.format(jt))
    plt.plot( fw_V_jt_seg[jt],  'o' )

    
for jt in range(0, n_jts):
    bk_V_jt_seg[jt].reverse()
    #print "bk_V_jt_seg[0]: {}".format( bk_V_jt_seg[jt] ) 
    plt.plot( bk_V_jt_seg[jt] , label='bk_vel_jt_{}'.format(jt))
    plt.plot( bk_V_jt_seg[jt],  'o')
    
plt.ylabel("velocity")
plt.xlabel("waypoint_number")
plt.legend()
plt.grid()
#plt.show()




## for dur difference between fwd and bkd 
fw_T =[]
bk_T =[]
for jt in range (0, n_jts):
    fw_seg_dur =[]
    bk_seg_dur =[]
    for seg in range (0, n_segs):    
        fw_seg_dur.append(  sum(fw_T_jt_seg[jt][seg]) )
        bk_seg_dur.append(  sum(bk_T_jt_seg[jt][seg]) )
    fw_T.append( fw_seg_dur)         
    bk_T.append( bk_seg_dur)         
        

fig = plt.figure()   
jt=0 # all joint are synchronized  
plt.plot( fw_T[jt], label='fw_dur')
plt.plot( fw_T[jt],  'o' )
bk_T[jt].reverse()
plt.plot( bk_T[jt], label='bk_dur')
plt.plot( bk_T[jt],  'o' )

plt.ylabel("duration - sec")
plt.xlabel("segment_number")
plt.legend()
plt.grid()
plt.show()

exit()





##### sample and plot traj using calculated jrk, times from previous section 
POS = []
VEL = []
ACC = []
JRK = [] 
TIM = []
T_SEG= []

for jt in range(0, n_jts): 
    Pos = []
    Vel = []
    Acc = []
    Jrk = [] 
    Tim = [] 
    T_seg= []
    # time and sampling 
    t_start = 0.0
    frq = 125.0
    T_seg=[ t_start]
    
    for seg in range(0, n_segs):
        J = J_jt_seg[jt][seg]
        T = T_jt_seg[jt][seg]
        V_init =  V_jt_seg[jt][seg]
        #T_seg: contains the start/end time of each segment
        T_seg.append(  sum(T) + T_seg[seg] )
        t_start= T_seg[seg]
        t= t_start
        while t < T_seg[seg+1] :
            pos, vel, acc, jrk = traj.sample_segment(t, t_start, path[jt][seg], V_init, J, T )    
            
            Pos.append(pos)
            Vel.append(vel) #* math.copysign(1, vel))
            Acc.append(acc)
            Jrk.append(jrk)
            Tim.append(t )
            t = t + (1.0/frq)
    POS.append(Pos)
    VEL.append(Vel)
    ACC.append(Acc)
    JRK.append(Jrk)
    TIM.append(Tim)
    T_SEG.append(T_seg)


#for i in range(0, n_jts):
#    V_jt_seg[i] = [ v*math.copysign(1, v) for v in V_jt_seg[i]]

print "dur: {}".format(      np.diff(   np.array(T_seg)   )         ) 
exit()


### plot pos, vel, acc, jrk. plot waypoints and estimated velocity as well to check if there is any difference 
fig, axes = plt.subplots(4, sharex=True)
for jt in range(0, n_jts):
    axes[0].plot( TIM[jt], POS[jt])
    axes[0].plot( T_SEG[jt], path[jt], 'o')
axes[0].set_ylabel('position')
axes[0].grid()


for jt in range(0, n_jts):
    axes[1].plot( TIM[jt], VEL[jt])
    axes[1].plot( T_SEG[jt], V_jt_seg[jt],  'o' )
axes[1].set_ylabel('velocity')
axes[1].grid()


for jt in range(0, n_jts):
    axes[2].plot( TIM[jt], ACC[jt])
axes[2].set_ylabel('acceleration')
axes[2].grid()

for jt in range(0, n_jts):
    axes[3].plot( TIM[jt], JRK[jt])
axes[3].set_ylabel('jerk')
axes[3].set_xlabel('Time')
axes[3].grid()


plt.legend()
plt.show()

#fig, axes = plt.subplots(n_jts, sharex=True)   
#for jt in range(0, n_jts):
#    axes[jt].plot( fw_V_jt_seg[jt])
#    axes[jt].plot( fw_V_jt_seg[jt],  'o' )
#    axes[jt].set_ylabel("velocity")
#axes[0].legend()
#  
#for jt in range(0, n_jts):
#    bk_V_jt_seg[jt].reverse()
#    axes[jt].plot( bk_V_jt_seg[jt])
#    axes[jt].plot( bk_V_jt_seg[jt],  'o' )
#    axes[jt].set_ylabel("velocity")
#axes[0].legend()
#plt.show()
