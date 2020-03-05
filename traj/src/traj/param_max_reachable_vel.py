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

import traj
import numpy as np
import math
    
##### find idx for stop points of path, points at which velocity direction changes   
def set_stp_pts_to_zero(path):
    seg_vel_dir = [ math.copysign(1, (p1-p0) )   for p0, p1 in zip(path[:-1], path[1:]) ]
    wpts_vel_dir = [seg_vel_dir[0]] + [ (p0+p1)/2   for p0, p1 in zip(seg_vel_dir[:-1], seg_vel_dir[1:]) ] + [seg_vel_dir[-1]]
    arr = np.array(wpts_vel_dir)
    stp_pts_idx = np.where(arr==0)[0]
    return wpts_vel_dir, stp_pts_idx
    

    
##### find max_ velocity per path in a certain direction 
def path_max_vel_per_dir(path, v_init, abs_max_pos, abs_max_vel, abs_max_acc, abs_max_jrk):     
    # get po dir at each waypoint and indx for stop point where velocity changes its direction
    wpts_vel_dir, stp_idx= traj.set_stp_pts_to_zero(path)
    stp_idx =  [id for id in stp_idx] 
    n_wpts = len(path) 
    max_vel_vec = [ abs(v_init)   ]  
    ### estimate max reachable velocity
    for wpt in range(0, n_wpts-1 ) :
        if wpt+1 in stp_idx:
            v_nxt = 0.0
        else:
            tj, ta, tv, v_nxt = traj.max_reachable_vel( abs(path[wpt] - path[wpt+1]), max_vel_vec[wpt], abs_max_pos, abs_max_vel, abs_max_acc, abs_max_jrk)
            #print "abs_pos_diff={}, v_forward={}, nxt_v={} ".format(abs(path[wpt] - path[wpt+1]), v_forward, v_frwrd)        
        max_vel_vec.append(  v_nxt )      
    return max_vel_vec
        

      
### find forward and backward vel per path (1dof path)
def find_max_estimated_vel_per_path(path, v_start, v_end, abs_max_pos, abs_max_vel, abs_max_acc, abs_max_jrk):
    Frwd_max_vel = path_max_vel_per_dir(path, v_start, abs_max_pos, abs_max_vel, abs_max_acc, abs_max_jrk)
    Bkwd_max_vel = path_max_vel_per_dir(path[::-1], v_end, abs_max_pos, abs_max_vel, abs_max_acc, abs_max_jrk)  #L[::-1]
    Bkwd_max_vel.reverse()
    
    ##check condition when v_start or v_end is not feasible: v_start > max_v_start calculated using the backward loop or Vs
    if Frwd_max_vel[0] > Bkwd_max_vel[0] or Frwd_max_vel[-1] < Bkwd_max_vel[-1]:
        raise ValueError("combination of v_start({}) & v_end({}) is not feasible".format(Frwd_max_vel[0], Bkwd_max_vel[-1] ) )                 
    # calcuate max_rechable_vels that grantee v_end at the end of the trajectory for this portion of traj
    estimated_vel =  [ min(v) for v in zip( Frwd_max_vel, Bkwd_max_vel)] 
    ## retrieve the direction at each way point 
    wpts_vel_dir, stp_idx= traj.set_stp_pts_to_zero(path)
    estimated_vel=[v*dir for v, dir in zip(estimated_vel, wpts_vel_dir) ]
    return Frwd_max_vel, Bkwd_max_vel, estimated_vel




######### find estimated max vel for traj, this is not time synchronized
def find_max_estimated_vel_per_ndof_path(path, v_start, v_end, abs_max_pos, abs_max_vel, abs_max_acc, abs_max_jrk):
    if len(path) != len(v_start) or len(path) != len(v_end):
        raise ValueError("Dimensions are not equal: len(path)={}, len(v_start)={}, len(v_end)={}".format(len(path) , len(v_start) , len(v_end) )   )          
    Estimated_vel = []    
    for pth in range(0, len(path) ):
        Frwd_max_vel, Bkwd_max_vel, estimated_vel =  find_max_estimated_vel_per_path(path[pth], v_start[pth], v_end[pth], abs_max_pos, abs_max_vel, abs_max_acc, abs_max_jrk)
        Estimated_vel.append( estimated_vel )
    return Estimated_vel
    
  
  
  












  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
    