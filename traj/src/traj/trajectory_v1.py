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
    
    
    
    
    
#def project_limits_onto_s(limits, function):
#    #print "function: {}".format( function )    
#    #print "limits: {}".format( limits )    
#    slope = np.abs(np.array(diff(function)).astype(np.float64).flatten())
#    #print "slope: {}".format( slope )    
#    limit_factor = limits / slope
##    print "limit_factor: {}".format( limit_factor )
#    #return min(limit_factor)
#    return limit_factor.tolist()

def project_limits_onto_s(limits, function):
    slope = np.abs(np.array( [diff(func) for func in function ] ).astype(np.float64).flatten())
    limit_factor = limits / slope
    return limit_factor.tolist()
#    return min(limit_factor)







def trajectory_for_path_v1(path, estimated_max_vel, max_positions,  max_velocities, max_accelerations, max_jerks):       
    path_function = parameterize_path(path)
    t = Symbol('t')
    s = path_function.independent_variable    
 
    n_wpts = len(path)
    n_segs = n_wpts - 1
    n_jts  = len(path[0])
    

############################ parameterize using estimated VEL ######################      
    traj_pos_seg_jt = []
    traj_vel_seg_jt = []
    traj_acc_seg_jt = []
    traj_jrk_seg_jt = []
    traj_times_jt = [  [0.0] for jt in range(n_jts)  ]
    print "traj_times_jt:{}".format(traj_times_jt)
    trajectory_boundaries= [0.0]
    
    
    s_estimated_vel= []
    
    for segment_i in range(len(path_function.functions)):
        fsegment = path_function.functions[segment_i]

        s0 = path_function.boundaries[segment_i]
        s1 = path_function.boundaries[segment_i + 1]

        p_start = np.array(fsegment.subs(s, 0.0)).astype(np.float64).flatten()
        p_end = np.array(fsegment.subs(s, s1 - s0)).astype(np.float64).flatten()
        print "\n\n>>>>>>>>>>>>>>>>>>>>>>>>> seg_{}: p_start={}, p_end={}, s1-s0={} <<<<<<<<<<<<<<<<<<<<<<".format(segment_i, p_start, p_end, s1-s0)
        print "\n\n>>> fsegment: \n{}".format(fsegment)
        
        
        # Project joint limits onto this segment's direction to get limits on s
        v_max = project_limits_onto_s(max_velocities, fsegment)
        a_max = project_limits_onto_s(max_accelerations, fsegment)
        j_max = project_limits_onto_s(max_jerks, fsegment)
               
       # Compute 7 segment profile for s as a function of time.
        this_segment_start_time = trajectory_boundaries[-1]
 
######################### option_1 #########################
#        if segment_i ==0:
#            s_init_vel = project_limits_onto_s( estimated_max_vel[segment_i], fsegment)
#            s_estimated_vel.append( s_init_vel )
#        
#        s_estimated_vel.append( project_limits_onto_s(estimated_max_vel[segment_i+1], fsegment) )
#        print ">>>s_estimated_vel: {}".format(s_estimated_vel)


########################## option_2 #########################
        s_init_vel = []
        s_nxt_vel  = []
        s_init_vel = project_limits_onto_s( estimated_max_vel[segment_i], fsegment)
        s_nxt_vel = project_limits_onto_s( estimated_max_vel[segment_i+1], fsegment)
        print "s_init_vel:{},  s_nxt_vel:{}".format(s_init_vel,  s_nxt_vel)

 

       
        
        ## for each joint 
        traj_pos_jt = []
        traj_vel_jt = []
        traj_acc_jt = []
        traj_jrk_jt = []
        traj_time_jt = [] 
        
        for jt in range(n_jts):
            print ">>> seg:{}, jt:{}".format(segment_i, jt)

########################## option_1
#            print ">>> pos_diff: {},  v0:{},  vnxt:{}".format( s1-s0, s_estimated_vel[segment_i][jt], s_estimated_vel[segment_i+1][jt] )
#            s_pos, s_vel, s_acc, s_jrk  = traj.fit_traj_segment(0, s1-s0, s_estimated_vel[segment_i][jt], s_estimated_vel[segment_i+1][jt], max_positions[jt], v_max[jt], a_max[jt], j_max[jt], t)


######################### option_2
            print ">>> pos_diff: {},  v0:{},  vnxt:{}".format( s1-s0, s_init_vel[jt], s_nxt_vel[jt] )
            s_pos, s_vel, s_acc, s_jrk  = traj.fit_traj_segment(0, s1-s0, s_init_vel[jt], s_nxt_vel[jt], max_positions[jt], v_max[jt], a_max[jt], j_max[jt], t)


######################### 
            jt_seg_start_time = traj_times_jt[jt][-1]
            # Substitute time profile for s into the path function to get trajectory as a function of t.
            traj_pos_ph = []
            traj_vel_ph = []
            traj_acc_ph = []
            traj_jrk_ph = []
            for function_i in range(len(s_pos.functions)):
                position_vs_t = fsegment.subs(s, s_pos.functions[function_i])
                velocity_vs_t = Matrix( [diff(pos, t) for pos in position_vs_t ]  )
                acceleration_vs_t = Matrix( [diff(vel, t) for vel in velocity_vs_t ] ) 
                jerk_vs_t = Matrix( [diff(acc, t) for acc in acceleration_vs_t ] )

                traj_times_jt[jt].append(s_pos.boundaries[function_i + 1] + jt_seg_start_time)               
                traj_pos_ph.append(position_vs_t[jt])
                traj_vel_ph.append(velocity_vs_t[jt])
                traj_acc_ph.append(acceleration_vs_t[jt])
                traj_jrk_ph.append(jerk_vs_t[jt])

            #print "\ntraj_times_jt{}: {}\n\n".format(jt, traj_times_jt[jt])         
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
        print "\n>> position_vs_t_Seg_{}: shp={} \n{}".format(segment_i, np.shape(traj_pos_seg_jt[segment_i]), traj_pos_seg_jt[segment_i])

    traj_pos_jt_seg = form_seg_jt_2_jt_seg( traj_pos_seg_jt )
    traj_vel_jt_seg = form_seg_jt_2_jt_seg( traj_vel_seg_jt )
    traj_acc_jt_seg = form_seg_jt_2_jt_seg( traj_acc_seg_jt )
    traj_jrk_jt_seg = form_seg_jt_2_jt_seg( traj_jrk_seg_jt )

#    print "\n>>> traj_pos_jt_seg: shp={} ".format( np.shape(traj_pos_jt_seg) )
    print "\n>>> traj_jrk_jt_seg: shp={} ".format( np.shape(traj_jrk_jt_seg) )
    print "\n>>> traj_times_jt: shp={} ".format( np.shape(traj_times_jt) )
    
    print "\n\n\n>>>>>>>>>>> Timing for each phase in each joint: <<<<<<<<<<<<<<<<<< "
    for seg in range(n_segs):
        for jt in range(n_jts):
            print "\n>>>traj_times_jt{}_seg{}: \n{}".format( jt, seg, traj_times_jt[jt][ 10*seg : 10*seg+10 ] )
    
    pos_piecewise_funcs=[] 
    vel_piecewise_funcs=[]
    acc_piecewise_funcs=[] 
    jrk_piecewise_funcs=[]
    for jt in range(n_jts):
        print "\n\n\n\n\n>>>jt={}".format( jt )
        print ">>>traj_jrk_jt_seg: \n{}".format( traj_jrk_jt_seg[jt] )
        print ">>>traj_times_jt: \n{}".format( traj_times_jt[jt]  )
        
        pos_piecewise_funcs.append(  PiecewiseFunction(traj_times_jt[jt],  traj_pos_jt_seg[jt]  , t)  )
        vel_piecewise_funcs.append(  PiecewiseFunction(traj_times_jt[jt],  traj_vel_jt_seg[jt]  , t)  )
        acc_piecewise_funcs.append(  PiecewiseFunction(traj_times_jt[jt],  traj_acc_jt_seg[jt]  , t)  )
        jrk_piecewise_funcs.append(  PiecewiseFunction(traj_times_jt[jt],  traj_jrk_jt_seg[jt]  , t)  )
        
    print "Done !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
    return (pos_piecewise_funcs, vel_piecewise_funcs, acc_piecewise_funcs, jrk_piecewise_funcs )




