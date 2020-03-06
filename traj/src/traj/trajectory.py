import numpy as np
from sympy import diff, Symbol

from piecewise_function import PiecewiseFunction
from parameterize_path import parameterize_path
import  traj


def project_limits_onto_s(limits, function, s_end):
    # print "\n>>diff_function: \n{}".format( np.array( diff(function) )  )
    slope0 = np.abs(np.array(diff(function).subs('s',0.0)).astype(np.float64).flatten())
    print "## slope_s=0: {}".format( slope0)

    slope1 = np.abs(np.array(diff(function).subs('s',s_end)).astype(np.float64).flatten())
    print "## slope_s=1: {}".format( slope1)

    slope = max( [slp for slp in slope0]+[slp for slp in slope1])
    limit_factor = limits / slope
    # print "\n## min(limit_factor): {}".format(min(limit_factor))
    return min(limit_factor)

def trajectory_for_path(path, max_velocities, max_accelerations, max_jerks):
    # path_function = parameterize_path(path)
    blend_radius = .1;
    path_function = traj.parameterize_path_with_blends(path, blend_radius)
    t = Symbol('t')
    s = path_function.independent_variable
    trajectory_position_functions = []
    trajectory_velocity_functions = []
    trajectory_acceleration_functions = []
    trajectory_jerk_functions = []
    trajectory_boundaries = [0.0]


    n_segs =  len(path_function.functions)
    print "n_segs ={}".format( n_segs )
    for seg in range( n_segs ):
        print "\n\n### seg: {}".format(seg)
        for jt in range( len(path[0]) ):
            print "\n=> q_{}(s) :  {}".format( jt, path_function.functions[seg][jt] )
        print "\n=> s0, s1 :  {}, {}".format( path_function.boundaries[seg], path_function.boundaries[seg+1] )
    


    ##any continous values for s_velocity 
    s_vel = [0.0, 0.4, 0.4, 0.2, 0.2, 0.0,      0.0, 0.0] 
    # s_vel = [0.0 for seg in range(n_segs+1)] 
    s_vel = [0.0, 0.5, 0.5,  0.0,      0.0, 0.0] 

    for segment_i in range(len(path_function.functions)):
        fsegment = path_function.functions[segment_i]
        print"\n\n ###################### seg: {} ###################### ".format(segment_i)
        s0 = path_function.boundaries[segment_i]
        s1 = path_function.boundaries[segment_i + 1]

        p_start = np.array(fsegment.subs(s, 0.0)).astype(np.float64).flatten()
        p_end = np.array(fsegment.subs(s, s1 - s0)).astype(np.float64).flatten()
        print(p_start, p_end)

        # Project joint limits onto this segment's direction to get limits on s
        v_max = project_limits_onto_s(max_velocities, fsegment, s1-s0)
        a_max = project_limits_onto_s(max_accelerations, fsegment, s1-s0)
        j_max = project_limits_onto_s(max_jerks, fsegment, s1-s0)

        # Compute 7 segment profile for s as a function of time.
        this_segment_start_time = trajectory_boundaries[-1]
        s_position, s_velocity, s_acceleration, s_jerk = traj.fit_traj_segment(0, s1-s0, s_vel[segment_i], s_vel[segment_i+1],  30.0, v_max, a_max, j_max, t)
        # def fit_traj_segment(p_start, p_end, v_start, v_end, p_max, v_max, a_max, j_max, independent_variable=Symbol('t')):

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

    return (PiecewiseFunction(trajectory_boundaries, trajectory_position_functions, t),
            PiecewiseFunction(trajectory_boundaries, trajectory_velocity_functions, t),
            PiecewiseFunction(trajectory_boundaries, trajectory_acceleration_functions, t),
            PiecewiseFunction(trajectory_boundaries, trajectory_jerk_functions, t))

