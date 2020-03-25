#!/usr/bin/env python
import traj
import numpy as np
import math
    

def set_velocities_at_stop_points_to_zero(path):
    ''' 
    this function finds the indices of the stop points along the path; the points at which the velocity direction changes 
    then it resets the velocities at these points to zeros   
    '''
    seg_vel_dir = [ math.copysign(1, (p1-p0) )   for p0, p1 in zip(path[:-1], path[1:]) ]
    wpts_vel_dir = [seg_vel_dir[0]] + [ (p0+p1)/2   for p0, p1 in zip(seg_vel_dir[:-1], seg_vel_dir[1:]) ] + [seg_vel_dir[-1]]
    arr = np.array(wpts_vel_dir)
    stp_pts_idx = np.where(arr==0)[0]
    return wpts_vel_dir, stp_pts_idx

    
def max_vel_at_each_waypoint_one_dof_path_case(path, v_init, abs_max_pos, abs_max_vel, abs_max_acc, abs_max_jrk):
    ''' 
    this function finds the maximum velocity at each waypoint along a 1-dof path "path", starting with initial velocity "v_init"  
    '''     
    wpts_vel_dir, stp_idx= traj.set_velocities_at_stop_points_to_zero(path)
    stp_idx =  [id for id in stp_idx] 
    n_wpts = len(path) 
    max_vel_vec = [ abs(v_init)   ]  
    for wpt in range(0, n_wpts-1 ) :
        if wpt+1 in stp_idx:
            v_nxt = 0.0
        else:
            tj, ta, tv, v_nxt = traj.max_reachable_vel_per_segment( abs(path[wpt] - path[wpt+1]), max_vel_vec[wpt], abs_max_pos, abs_max_vel, abs_max_acc, abs_max_jrk)
        max_vel_vec.append(  v_nxt )      
    return max_vel_vec
        
 
def reachable_vel_at_each_waypoint_one_dof_path_case(path, v_start, v_end, abs_max_pos, abs_max_vel, abs_max_acc, abs_max_jrk):
    ''' 
    this function finds the estimated reachable velocity at each waypoint along a 1-dof path "path", with starting velocity "v_init", a final velocity "v_end"   
    taking into considereation vel/acc/jrk constraints. this idea is the same idea behind the TOPP-RA paper: "A New Approach to Time-Optimal Path Parameterization
    based on Reachability Analysis [H. pham 2018]
    '''  
    frwd_max_vel = max_vel_at_each_waypoint_one_dof_path_case(path, v_start, abs_max_pos, abs_max_vel, abs_max_acc, abs_max_jrk)
    bkwd_max_vel = max_vel_at_each_waypoint_one_dof_path_case(path[::-1], v_end, abs_max_pos, abs_max_vel, abs_max_acc, abs_max_jrk)  #L[::-1]
    bkwd_max_vel.reverse()
    # check condition when v_start or v_end is not feasible: v_start > max_v_start calculated using the backward loop or Vs
    if frwd_max_vel[0] > bkwd_max_vel[0] or frwd_max_vel[-1] < bkwd_max_vel[-1]:
        raise ValueError("combination of v_start({}) & v_end({}) is not feasible".format(frwd_max_vel[0], bkwd_max_vel[-1] ) )                 
    # calcuate max_rechable_vels that grantee v_end at the end of the trajectory for this portion of traj
    estimated_vel =  [ min(v) for v in zip( frwd_max_vel, bkwd_max_vel)] 
    # retrieve the direction at each way point 
    wpts_vel_dir, stp_idx= traj.set_velocities_at_stop_points_to_zero(path)
    estimated_vel=[v*dir for v, dir in zip(estimated_vel, wpts_vel_dir) ]
    return frwd_max_vel, bkwd_max_vel, estimated_vel


def reachable_vel_at_each_waypoint_multi_dof_path_case(path, v_start, v_end, abs_max_pos, abs_max_vel, abs_max_acc, abs_max_jrk):
    ''' 
    this function finds the estimated reachable velocity at each waypoint along a n-dof path "path", with starting velocity "v_init", a final velocity "v_end"   
    taking into considereation vel/acc/jrk constraints. this idea is the same idea behind the TOPP-RA paper: "A New Approach to Time-Optimal Path Parameterization
    based on Reachability Analysis [H. pham 2018]
    paper link: https://www.researchgate.net/publication/318671280_A_New_Approach_to_Time-Optimal_Path_Parameterization_Based_on_Reachability_Analysis
    '''  
    if len(path) != len(v_start) or len(path) != len(v_end):
        raise ValueError("Dimensions are not equal: len(path)={}, len(v_start)={}, len(v_end)={}".format(len(path) , len(v_start) , len(v_end) )   )          
    reachable_vel = []    
    for pth in range(0, len(path) ):
        frwd_max_vel, bkwd_max_vel, estimated_vel =  reachable_vel_at_each_waypoint_one_dof_path_case(path[pth], v_start[pth], v_end[pth], abs_max_pos, abs_max_vel, abs_max_acc, abs_max_jrk)
        reachable_vel.append( estimated_vel )
    return reachable_vel
