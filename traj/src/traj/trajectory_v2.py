import numpy as np
from sympy import diff, Symbol
from piecewise_function import PiecewiseFunction
from parameterize_path import parameterize_path
import traj
import rospy


def project_limits_onto_s(limits, function, s_end):
    slope = np.abs(np.array(diff(function)).astype(np.float64).flatten())
    limit_factor = limits / slope
    return min(limit_factor)


def trajectory_for_path_v2(path, v_start, v_end,
                           max_velocities, max_accelerations, max_jerks):
    path_function = parameterize_path(path)
    t = Symbol('t')
    s = path_function.independent_variable
    # check n_jts, n_segs
    n_segs = len(path_function.functions)

    # step 1: find Max Forward velocity
    s_fw_vel = []
    for seg in range(n_segs):
        fsegment = path_function.functions[seg]
        s0 = path_function.boundaries[seg]
        s1 = path_function.boundaries[seg + 1]
        # Project joint limits onto this segment's direction to get limits on s
        v_max = project_limits_onto_s(max_velocities, fsegment, s1-s0)
        a_max = project_limits_onto_s(max_accelerations, fsegment, s1-s0)
        j_max = project_limits_onto_s(max_jerks, fsegment, s1-s0)
        if seg == 0:
            s_v_start = project_limits_onto_s(v_start, fsegment, s1-s0)
            s_fw_vel.append(s_v_start)
        tj, ta, tv, s_v_nxt = traj.max_reachable_vel(s1-s0, s_fw_vel[seg],
                                                     30.0, j_max, a_max, j_max)
        s_fw_vel.append(s_v_nxt)
    rospy.logdebug("\n>>> s_fw_vel: \n {}".format(s_fw_vel))

    # step 2: find Max Backward velocity
    s_bk_vel = []
    for seg in range(n_segs):
        fsegment = path_function.functions[n_segs - seg - 1]
        s0 = path_function.boundaries[n_segs - seg - 1]
        s1 = path_function.boundaries[n_segs - seg]
        # Project joint limits onto this segment's direction to get limits on s
        v_max = project_limits_onto_s(max_velocities, fsegment, s1-s0)
        a_max = project_limits_onto_s(max_accelerations, fsegment, s1-s0)
        j_max = project_limits_onto_s(max_jerks, fsegment, s1-s0)
        if seg == 0:
            s_v_end = project_limits_onto_s(v_end, fsegment, s1-s0)
            s_bk_vel.append(s_v_end)
        tj, ta, tv, s_v_nxt = traj.max_reachable_vel(
            s1-s0, s_bk_vel[seg], 30.0, v_max, a_max, j_max)
        s_bk_vel.append(s_v_nxt)
    s_bk_vel.reverse()
    rospy.logdebug("\n>>> s_bk_vel: \n {}".format(s_bk_vel))

    # step 3: find final max reachable vel
    # check condition when v_start or v_end is not feasible:
    # v_start > max_v_start calculated using the backward loop or Vs
    if s_fw_vel[0] > s_bk_vel[0] or s_bk_vel[-1] > s_fw_vel[-1]:
        raise ValueError("combination of v_start({}) & v_end({})"
                         "is not feasible".format(s_fw_vel[0], s_bk_vel[-1]))
    # calcuate max_rechable_vels that grantee v_end at the end of
    # the trajectory for this portion of traj
    s_estimated_vel = [min(fw, bk) for fw, bk in zip(s_fw_vel, s_bk_vel)]
    rospy.logdebug("\n>>> s_estimated_vel: \n {}".format(s_estimated_vel))
    # step 4: use the estimated max reachable velocity
    trajectory_position_functions = []
    trajectory_velocity_functions = []
    trajectory_acceleration_functions = []
    trajectory_jerk_functions = []
    trajectory_boundaries = [0.0]
    for segment_i in range(len(path_function.functions)):
        fsegment = path_function.functions[segment_i]
        rospy.logdebug("\n\n >>>>>>>>> seg: {} ".format(segment_i))
        s0 = path_function.boundaries[segment_i]
        s1 = path_function.boundaries[segment_i + 1]
        p_start = np.array(fsegment.subs(s, 0.0)).astype(np.float64).flatten()
        p_end = np.array(
                fsegment.subs(s, s1 - s0)).astype(np.float64).flatten()
        rospy.logdebug((p_start, p_end))

        # Project joint limits onto this segment's direction to get limits on s
        v_max = project_limits_onto_s(max_velocities, fsegment, s1-s0)
        a_max = project_limits_onto_s(max_accelerations, fsegment, s1-s0)
        j_max = project_limits_onto_s(max_jerks, fsegment, s1-s0)
        # Compute 7 segment profile for s as a function of time.
        this_segment_start_time = trajectory_boundaries[-1]
        s_position, s_velocity, s_acceleration, s_jerk = traj.fit_traj_segment(
            0, s1-s0, s_estimated_vel[segment_i], s_estimated_vel[segment_i+1],
            30.0, v_max, a_max, j_max)

        # Substitute time profile for s into the path function to get
        # trajectory as a function of t.
        for function_i in range(len(s_position.functions)):
            position_vs_t = fsegment.subs(s, s_position.functions[function_i])
            velocity_vs_t = diff(position_vs_t, t)
            acceleration_vs_t = diff(velocity_vs_t, t)
            jerk_vs_t = diff(acceleration_vs_t, t)
            trajectory_position_functions.append(position_vs_t)
            trajectory_velocity_functions.append(velocity_vs_t)
            trajectory_acceleration_functions.append(acceleration_vs_t)
            trajectory_jerk_functions.append(jerk_vs_t)
            trajectory_boundaries.append(
                s_position.boundaries[function_i + 1] +
                this_segment_start_time)

    return (PiecewiseFunction(trajectory_boundaries,
                              trajectory_position_functions, t),
            PiecewiseFunction(trajectory_boundaries,
                              trajectory_velocity_functions, t),
            PiecewiseFunction(trajectory_boundaries,
                              trajectory_acceleration_functions, t),
            PiecewiseFunction(trajectory_boundaries,
                              trajectory_jerk_functions, t))
