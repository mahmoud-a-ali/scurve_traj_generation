import numpy as np
from sympy import diff, Symbol

from piecewise_function import PiecewiseFunction
from parameterize_path import parameterize_path

from . import seven_segment_type3


def project_limits_onto_s(joint_limits, q_of_s):
    """

    For given max joint limits, projects them onto a linear function q(s) to
    give the max value of s.

    This function works for position, velocity, jerk, and acceleration limits.
    It assumes that the limits on each joint are symmetric; i.e. that
    q_min = -q_max.

    Args:
        joint_limits: maximum linear (jerk or acceleration or velocity) for each
          joint in an array.
        q_of_s: Sympy function that maps from s to a vector of joint values.
            Must be linear for this function to work.

    Returns:
        Maximum value for s(t) (or the first/second/third derivative of s(t)
        depending on whether you passed in velocity/acceleration/jerk limits).
    """
    slope = np.abs(np.array(diff(q_of_s)).astype(np.float64).flatten())
    limit_factor = joint_limits / slope

    return min(limit_factor)


def trajectory_for_path(path, max_velocities, max_accelerations, max_jerks):
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

        s0 = path_function.boundaries[segment_i]
        s1 = path_function.boundaries[segment_i + 1]

        p_start = np.array(fsegment.subs(s, 0.0)).astype(np.float64).flatten()
        p_end = np.array(fsegment.subs(s, s1 - s0)).astype(np.float64).flatten()

        # Project joint limits onto this segment's direction to get limits on s
        v_max = project_limits_onto_s(max_velocities, fsegment)
        a_max = project_limits_onto_s(max_accelerations, fsegment)
        j_max = project_limits_onto_s(max_jerks, fsegment)

        # Compute 7 segment profile for s as a function of time.
        this_segment_start_time = trajectory_boundaries[-1]
        s_jerk = seven_segment_type3.fit(
            0, s1 - s0, v_max, a_max, j_max, t)
        s_acceleration = s_jerk.integrate(0.0)
        s_velocity = s_acceleration.integrate(0.0)
        s_position = s_velocity.integrate(0.0)

        # Substitute time profile for s into the path function to get trajectory as a function of t.
        for function_i in range(len(s_position.functions)):
            position_vs_t = fsegment.subs(s, s_position.functions[function_i])
            velocity_vs_t = diff(position_vs_t)
            acceleration_vs_t = diff(velocity_vs_t)
            jerk_vs_t = diff(acceleration_vs_t)
            trajectory_position_functions.append(position_vs_t)
            trajectory_velocity_functions.append(velocity_vs_t)
            trajectory_acceleration_functions.append(acceleration_vs_t)
            trajectory_jerk_functions.append(jerk_vs_t)
            trajectory_boundaries.append(s_position.boundaries[function_i + 1] + this_segment_start_time)

    return (PiecewiseFunction(trajectory_boundaries, trajectory_position_functions, t),
            PiecewiseFunction(trajectory_boundaries, trajectory_velocity_functions, t),
            PiecewiseFunction(trajectory_boundaries, trajectory_acceleration_functions, t),
            PiecewiseFunction(trajectory_boundaries, trajectory_jerk_functions, t))
