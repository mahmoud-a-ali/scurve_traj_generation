import numpy as np
from sympy import diff, Symbol

from piecewise_function import PiecewiseFunction
from parameterize_path import parameterize_path
import  traj


def project_limits_onto_s(limits, function):
    slope = np.abs(np.array(diff(function)).astype(np.float64).flatten())
    limit_factor = limits / slope

    return min(limit_factor)

def trajectory_for_path_v0(path, common_vel, max_positions, max_velocities, max_accelerations, max_jerks):
    s_vel= []    
    path_function = parameterize_path(path)
    t = Symbol('t')
    s = path_function.independent_variable
    trajectory_position_functions = []
    trajectory_velocity_functions = []
    trajectory_acceleration_functions = []
    trajectory_jerk_functions = []
    trajectory_boundaries = [0.0]
    for segment_i in range(len(path_function.functions)):
        fsegment = path_function.functions[segment_i]
        print "\n\n>>> fsegment: \n{}".format(fsegment)

        # Project joint limits onto this segment's direction to get limits on s
        v_max = project_limits_onto_s(max_velocities, fsegment)
        a_max = project_limits_onto_s(max_accelerations, fsegment)
        j_max = project_limits_onto_s(max_jerks, fsegment)


        s0 = path_function.boundaries[segment_i]
        s1 = path_function.boundaries[segment_i + 1]

        p_start = np.array(fsegment.subs(s, 0.0)).astype(np.float64).flatten()
        p_end = np.array(fsegment.subs(s, s1 - s0)).astype(np.float64).flatten()
#        print(p_start, p_end)


##### option_1 ###########
#        if segment_i ==0:
#            s_vel.append( project_limits_onto_s(common_vel[segment_i], fsegment) )
# 
#        s_vel.append( project_limits_onto_s(common_vel[segment_i+1], fsegment) )
#        print "s_vel: {}".format(s_vel)
#        # Compute 7 segment profile for s as a function of time.
#        this_segment_start_time = trajectory_boundaries[-1]
#        s_position, s_velocity, s_acceleration, s_jerk = traj.fit_traj_segment(0, s1-s0, s_vel[segment_i], s_vel[segment_i+1], s1-s0, v_max, a_max, j_max, t)


##### option_2 ###########
        s_start_vel=  project_limits_onto_s(common_vel[segment_i], fsegment) 
        s_end_vel= project_limits_onto_s(common_vel[segment_i+1], fsegment) 
        print ">>>> s_start_vel: {}, s_end_vel:{}".format(s_start_vel, s_end_vel)
        # Compute 7 segment profile for s as a function of time.
        this_segment_start_time = trajectory_boundaries[-1]
        s_position, s_velocity, s_acceleration, s_jerk = traj.fit_traj_segment(0, s1-s0, s_start_vel, s_end_vel, s1-s0, v_max, a_max, j_max, t)

        

        # Substitute time profile for s into the path function to get trajectory as a function of t.
        for function_i in range(len(s_position.functions)):
            position_vs_t = fsegment.subs(s, s_position.functions[function_i])
            velocity_vs_t = diff(position_vs_t, t)
            acceleration_vs_t = diff(velocity_vs_t, t)
            jerk_vs_t = diff(acceleration_vs_t, t)
            
            trajectory_position_functions.append(position_vs_t)
            trajectory_velocity_functions.append(velocity_vs_t)
            trajectory_acceleration_functions.append(acceleration_vs_t)
            trajectory_jerk_functions.append(jerk_vs_t)
            trajectory_boundaries.append(s_position.boundaries[function_i + 1] + this_segment_start_time)
            print "\n>> position_vs_t_Seg_{}_ph_{}: shp={} \n{}".format(segment_i, function_i, np.shape(position_vs_t), position_vs_t)

    return (PiecewiseFunction(trajectory_boundaries, trajectory_position_functions, t),
            PiecewiseFunction(trajectory_boundaries, trajectory_velocity_functions, t),
            PiecewiseFunction(trajectory_boundaries, trajectory_acceleration_functions, t),
            PiecewiseFunction(trajectory_boundaries, trajectory_jerk_functions, t))
