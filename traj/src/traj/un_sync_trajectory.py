import numpy as np
from sympy import diff, Symbol

from piecewise_function import PiecewiseFunction
from parameterize_path import parameterize_path
import  traj
from sympy import Matrix


def reverse_list( lst ):
    arr = np.array( lst )
    arr = np.rot90(arr).tolist()
    arr.reverse()
    return arr

def form_seg_jt_2_jt_seg(lst):
    #n_segs  = len( lst )
    #n_jts   = len( lst[0] )
    #n_phs   = len( lst[0][0] ) 
    arr = np.array(lst)
    arr = np.rot90(arr)
    lst = np.reshape(arr,   ( len( lst[0] ), len(lst)*len(lst[0][0]) )   ).tolist()
    lst.reverse()   
    return lst    
    
def project_limits_onto_s(limits, function):
    #print "function: {}".format( function )    
    #print "limits: {}".format( limits )    
    slope = np.abs(np.array(diff(function)).astype(np.float64).flatten())
    #print "slope: {}".format( slope )    
    limit_factor = limits / slope
    #return min(limit_factor)
    return limit_factor.tolist()


def un_sync_trajectory_for_path(path, v_start, v_end, max_velocities, max_accelerations, max_jerks):       
    path_function = parameterize_path(path)
    t = Symbol('t')
    s = path_function.independent_variable    

    n_segs = len(path_function.functions)
    n_wpts = n_segs +1
    n_jts  = len(v_start)
    
    
############################ FW VEL ######################   
    s_fw_vel = []  
    for seg in range(n_segs):
        fsegment = path_function.functions[seg]
        s0 = path_function.boundaries[seg]
        s1 = path_function.boundaries[seg + 1]
        
        # Project joint limits onto this segment's direction to get limits on s
        v_max = project_limits_onto_s(max_velocities, fsegment)
        a_max = project_limits_onto_s(max_accelerations, fsegment)
        j_max = project_limits_onto_s(max_jerks, fsegment)

        if seg == 0:
            s_v_start = project_limits_onto_s(v_start, fsegment)
            s_fw_vel.append( s_v_start)
            
        s_nxt_vel = []
        for jt in range(n_jts):
            print "\n>> s1-s0, s_fw_vel[seg][jt], 30.0, v_max[jt], a_max[jt], j_max[jt] "
            print s1-s0, s_fw_vel[seg][jt], 30.0, v_max[jt], a_max[jt], j_max[jt]
            tj, ta, tv, s_v_nxt = traj.max_reachable_vel( s1-s0, s_fw_vel[seg][jt], 30.0, v_max[jt], a_max[jt], j_max[jt])
            s_nxt_vel.append( s_v_nxt )   
        s_fw_vel.append( s_nxt_vel )
    print "\n>>> s_fw_vel: \n {} \n\n".format( s_fw_vel)
 
   
   
   
   
############################ BK VEL ######################   
    s_bk_vel = []
    for seg in range(n_segs):
        fsegment = path_function.functions[n_segs - seg -1 ]
        s0 = path_function.boundaries[n_segs - seg - 1]
        s1 = path_function.boundaries[n_segs - seg ]
        
        # Project joint limits onto this segment's direction to get limits on s
        v_max = project_limits_onto_s(max_velocities, fsegment)
        a_max = project_limits_onto_s(max_accelerations, fsegment)
        j_max = project_limits_onto_s(max_jerks, fsegment)
    
        if seg == 0:
            s_v_end = project_limits_onto_s(v_end, fsegment)
            s_bk_vel.append( s_v_end)

        s_nxt_vel = []
        for jt in range(n_jts):
            print "/n>> s1-s0, s_bk_vel[seg][jt], 30.0, v_max[jt], a_max[jt], j_max[jt] "
            print  s1-s0, s_bk_vel[seg][jt], 30.0, v_max[jt], a_max[jt], j_max[jt] 
            tj, ta, tv, s_v_nxt = traj.max_reachable_vel( s1-s0, s_bk_vel[seg][jt], 30.0, v_max[jt], a_max[jt], j_max[jt] )
            s_nxt_vel.append( s_v_nxt )   
        s_bk_vel.append( s_nxt_vel )
 
    s_bk_vel.reverse()
    print "\n>>> s_bk_vel: \n {}".format( s_bk_vel)
    



   
############################ estimated VEL ######################   
    ##check condition when v_start or v_end is not feasible: v_start > max_v_start calculated using the backward loop or Vs
    for jt in range(n_jts):
        if s_fw_vel[0][jt] > s_bk_vel[0][jt] or s_bk_vel[-1][jt] > s_fw_vel[-1][jt] :
            raise ValueError("combination of v_start({}) & v_end({}) for jt_{}is not feasible".format(s_fw_vel[0][jt], s_bk_vel[-1][jt], jt ) )                 
    # calcuate max_rechable_vels that grantee v_end at the end of the trajectory for this portion of traj
    s_estimated_vel = []
    for wpt in range(n_wpts):
        s_seg_vel= [ min(v) for v in zip( s_fw_vel[wpt], s_bk_vel[wpt])]
        s_estimated_vel.append( s_seg_vel  )
    print "\n>>> s_estimated_vel: \n {}\n\n\n".format( s_estimated_vel)
    
    
############################ parameterize using estimated VEL ######################      
    traj_pos_seg_jt = []
    traj_vel_seg_jt = []
    traj_acc_seg_jt = []
    traj_jrk_seg_jt = []
    traj_times_jt = [  [0.0] for jt in range(n_jts)  ]
    print "traj_times_jt:{}".format(traj_times_jt)
    trajectory_boundaries= [0.0]
    for segment_i in range(len(path_function.functions)):
        fsegment = path_function.functions[segment_i]

        s0 = path_function.boundaries[segment_i]
        s1 = path_function.boundaries[segment_i + 1]

        p_start = np.array(fsegment.subs(s, 0.0)).astype(np.float64).flatten()
        p_end = np.array(fsegment.subs(s, s1 - s0)).astype(np.float64).flatten()
        print ">>>>>>>>>>>>>> seg_{}: p_start={}, p_end={}, s1-s0={}".format(segment_i, p_start, p_end, s1-s0)

        # Project joint limits onto this segment's direction to get limits on s
        v_max = project_limits_onto_s(max_velocities, fsegment)
        a_max = project_limits_onto_s(max_accelerations, fsegment)
        j_max = project_limits_onto_s(max_jerks, fsegment)
        
#        print "\n>>> s_v_max ={}".format(v_max)

        # Compute 7 segment profile for s as a function of time.
        this_segment_start_time = trajectory_boundaries[-1]
        
        
        ## for each joint 
        traj_pos_jt = []
        traj_vel_jt = []
        traj_acc_jt = []
        traj_jrk_jt = []
        traj_time_jt = [] 
        
        for jt in range(n_jts):
            s_pos, s_vel, s_acc, s_jrk  = traj.fit_traj_segment(0, s1-s0, s_estimated_vel[segment_i][jt], s_estimated_vel[segment_i+1][jt], 30.0, v_max[jt], a_max[jt], j_max[jt], t)
            print "\n\n\n seg:{}, jt:{}".format(segment_i, jt)
            print "\n>>> pos_diff: {},  v0:{},  vnxt:{}".format( s1-s0, s_estimated_vel[segment_i][jt], s_estimated_vel[segment_i+1][jt] )
#            print "\n>>> type(s_pos.functions) : {}".format( type(s_pos.functions) )
#            print "\n>>> type(s_pos.functions[0]) : {}".format( type(s_pos.functions[0]) )
#            print "\n>>> s_pos.boundaries : {}".format( s_pos.boundaries )
            jt_seg_start_time = traj_times_jt[jt][-1]
#            print "jt_seg_start_time_jt{}: {}".format(jt, jt_seg_start_time)            
            # Substitute time profile for s into the path function to get trajectory as a function of t.
            traj_pos_ph = []
            traj_vel_ph = []
            traj_acc_ph = []
            traj_jrk_ph = []
            for function_i in range(len(s_pos.functions)):
                position_vs_t = fsegment.subs(s, s_pos.functions[function_i])
                velocity_vs_t = diff(position_vs_t, t)
                acceleration_vs_t = diff(velocity_vs_t, t)
                jerk_vs_t = diff(acceleration_vs_t, t)
#                print "\n\n\ns_pos.boundaries[function_i + 1]:{} ".format( s_pos.boundaries[function_i + 1] ) 
#                print " jt_seg_start_time[jt]: {}".format( jt_seg_start_time) 
                traj_times_jt[jt].append(s_pos.boundaries[function_i + 1] + jt_seg_start_time)
                #print "position_vs_t: {}".format( position_vs_t[jt] )
#                print "\n>>> len(position_vs_t): {}".format( len(position_vs_t) )
#                print "\n>>> len(velocity_vs_t): {}".format( len(velocity_vs_t) )
#                print "\n>>> len(acceleration_vs_t): {}".format( len(acceleration_vs_t) )
#                print "\n>>> len(jerk_vs_t): {}".format( len(jerk_vs_t) )
                
                
                traj_pos_ph.append(position_vs_t[jt])
#                print "\n>>> type(traj_pos_ph) : {}".format( type(traj_pos_ph) )
#                print "\n>>> type(traj_pos_ph[0]) : {}".format( type(traj_pos_ph[0]) )
                traj_vel_ph.append(velocity_vs_t[jt])
                traj_acc_ph.append(acceleration_vs_t[jt])
                traj_jrk_ph.append(jerk_vs_t[jt])
#                traj_boundaries.append(s_pos.boundaries[function_i + 1] + this_segment_start_time)
    
#            print "\ntraj_times_jt{}: {}\n\n".format(jt, traj_times_jt[jt])         
            traj_pos_jt.append( traj_pos_ph ) 
            traj_vel_jt.append( traj_vel_ph )
            traj_acc_jt.append( traj_acc_ph ) 
            traj_jrk_jt.append( traj_jrk_ph ) 
        
        trajectory_boundaries.append(s_pos.boundaries[function_i + 1] + this_segment_start_time ) 
            
        traj_pos_seg_jt.append( traj_pos_jt ) 
        traj_vel_seg_jt.append( traj_vel_jt ) 
        traj_acc_seg_jt.append( traj_acc_jt ) 
        traj_jrk_seg_jt.append( traj_jrk_jt ) 
        #    traj_times_seg_jt.append( traj_pos_ph ) 
    

#    print "\n>>> traj_pos_seg_jt: type, len: {} {}, {} {}, {} {}, {}".format( type(traj_pos_seg_jt), len(traj_pos_seg_jt) , 
#    type(traj_pos_seg_jt[0]), len(traj_pos_seg_jt[0]), type(traj_pos_seg_jt[0][0]),  len(traj_pos_seg_jt[0][0]), type(traj_pos_seg_jt[0][0][0])  )

#    lst= [     [ [1,2,3,4], [11,12,13,14] ],       [ [5,6,7,8],[15,16,17,18] ],      [ [9,99,999,9999], [19,199,1999,19999] ]     ]

#    print "\n>>> trajectory_position_functions: {}".format([fun for fun in trajectory_position_functions])
#    print "\n>>> trajectory_velocity_functions: {}".format(fun for fun in trajectory_velocity_functions)
#    print "\n>>> trajectory_acceleration_functions: {}".format(fun for fun in trajectory_acceleration_functions)
#    print "\n>>> trajectory_jerk_functions: {}".format(fun for fun in trajectory_jerk_functions)
#    print "\n>>> trajectory_boundaries: {}".format(fun for fun in trajectory_boundaries)
    traj_pos_jt_seg = form_seg_jt_2_jt_seg( traj_pos_seg_jt )
    traj_vel_jt_seg = form_seg_jt_2_jt_seg( traj_vel_seg_jt )
    traj_acc_jt_seg = form_seg_jt_2_jt_seg( traj_acc_seg_jt )
    traj_jrk_jt_seg = form_seg_jt_2_jt_seg( traj_jrk_seg_jt )

#    print "\n>>> traj_pos_jt_seg: type, len: {} {}, {} {}, {} ".format( type(traj_pos_jt_seg), len(traj_pos_jt_seg) , 
#    type(traj_pos_jt_seg[0]), len(traj_pos_jt_seg[0]), type(traj_pos_jt_seg[0][0])  )


    pos_piecewise_funcs=[] 
    vel_piecewise_funcs=[]
    acc_piecewise_funcs=[] 
    jrk_piecewise_funcs=[]
    for jt in range(n_jts):
        print "\n\n\n\n\n>>>jt={}".format( jt )
        print "\n>>>traj_acc_jt_seg={}".format( traj_acc_jt_seg[jt] )
        print "\n traj_times_jt    ={}".format( traj_times_jt[jt]   )
        
        pos_piecewise_funcs.append(  PiecewiseFunction(traj_times_jt[jt],  traj_pos_jt_seg[jt]  , t)  )
        vel_piecewise_funcs.append(  PiecewiseFunction(traj_times_jt[jt],  traj_vel_jt_seg[jt]  , t)  )
        acc_piecewise_funcs.append(  PiecewiseFunction(traj_times_jt[jt],  traj_acc_jt_seg[jt]  , t)  )
        jrk_piecewise_funcs.append(  PiecewiseFunction(traj_times_jt[jt],  traj_jrk_jt_seg[jt]  , t)  )
        
    print "Done !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
    return (pos_piecewise_funcs, vel_piecewise_funcs, acc_piecewise_funcs, jrk_piecewise_funcs )




