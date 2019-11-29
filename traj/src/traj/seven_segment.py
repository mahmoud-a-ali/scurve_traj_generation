from sympy import Matrix, Piecewise, Symbol

from . import piecewise_function

def fit_seven_segment(p_start, p_end, j_max, a_max, v_max):
    """
    Find the optimal seven segment trajectory for zero start and end velocities, and the given
    start and end positions.

    Follows the nomenclature and approach of

        Herrera-Aguilar, Ignacio, and Daniel Sidobre. "Soft motion trajectory planning and
        control for service manipulator robot." Workshop on Physical Human-Robot Interaction in
        Anthropic Domains at IROS. 2006.
    """
    assert(a_max > 0.0)
    assert(j_max > 0.0)
    assert(v_max > 0.0)

    # Maximum amount of time we can spend at any of our limit conditions before we violate the
    # next higher limit condition.
    T_jmax = a_max / j_max
    T_amax = v_max / a_max - a_max / j_max

    if T_amax < 0.0:
        # Using max positive jerk and then max negative jerk, we don't have time to
        # reach max acceleration before reaching max velocity. To account for this,
        # we adjust the max acceleration down to what we can actually reach on our way
        # to the max velocity, and we adjust the max time spent in the max jerk limited
        # section to just reach the new max acceleration.
        T_amax = 0.0
        T_jmax = (v_max / j_max)**0.5
        print(T_jmax, j_max)
        a_max = T_jmax * j_max

    # Compute the minimum distance that each case can travel. D_thr1 is the minimum distance for
    # a trajectory that hits both max acceleration. D_thr2 is the minimum distance for a
    # trajectory that hits max acceleration but not max velocity.
    #
    # Q: shouldn't there also be a case where v_max is reached but a_max is not, depending on the
    #    relation between v_max, a_max, and j_max?
    D_thr1 = (a_max * v_max) / j_max + v_max**2 / a_max
    D_thr2 = 2.0 * a_max**3 / j_max**2

    D = p_end - p_start
    if D >= D_thr1:
        # We hit both v_max and a_max
        print('Case 1')
        T_j = T_jmax
        T_a = T_amax
        T_v = (D - D_thr1) / v_max
    elif D > D_thr2:
        print('Case 2')
        # We hit a_max but not v_max
        T_v = 0.0
        T_j = T_jmax
        T_a = (a_max**2 / (4.0 * j_max) + D / a_max)**0.5 - 1.5 * a_max / j_max
    else:
        print('Case 3')
        # We hit neither a_max nor v_max
        T_v = 0.0
        T_a = 0.0
        T_j = (D / (2.0 * j_max))**(1.0 / 3.0)

    segment_jerks_and_durations = [(j_max, T_j), (0.0, T_a), (-j_max, T_j), (0.0, T_v), (-j_max,
        T_j), (0.0, T_a), (j_max, T_j)]
    segments = []
    p0 = p_start
    v0 = 0.0
    a0 = 0.0
    times = [0.0]
    jerk_functions = []
    acceleration_functions = []
    velocity_functions = []
    position_functions = []
    t = Symbol('t')
    for j0, T in segment_jerks_and_durations:
        times.append(times[-1] + T)
        j = j0
        a = (integrate(j, t) + a0).subs(t - times[-1])
        v = integrate(a, t) + v0
        p = integrate(v, t) + p0
        jerk_functions.append(j)
        acceleration_functions.append(a)
        velocity_functions.append(v)
        position_functions.append(p)
    position = PiecewiseLinear(times, position_functions)
    velocity = PiecewiseLinear(times, velocity_functions)
    acceleration = PiecewiseLinear(times, acceleration_functions)
    jerk = PiecewiseLinear(times, jerk_functions)
    return position, velocity, acceleration, jerk

def test_fit_seven_segment(p_start, p_end, j_max, a_max, v_max):
    position, velocity, jerk, acceleration = fit_seven_segment(
            p_start, p_end, j_max, a_max, v_max)

    t_start = trajectory.segment_boundary_times[0]
    t_end = trajectory.segment_boundary_times[-1]
    p_start_actual = trajectory.pos(t_start)
    v_start_actual = trajectory.vel(t_start)
    p_end_actual = trajectory.pos(t_end)
    v_end_actual = trajectory.vel(t_end)

if __name__ == '__main__':
    # j_max, a_max, v_max, p_start, p_end
    tests = [
            (0.1, 3.0, 0.1, 0.0, 30.0), # Reach max velocity bot not max acceleration
            (0.1, 3.0, 6.0, 0.0, 30.0), # Reach max acceleration but not max velocity
            (0.1, 2.3, 3.1, 0.0, 0.4), # Don't reach max velocity or max acceleration
            (0.1, 0.4, 2.0, 0.0, 30.0), # Reach max acceleration and also velocity
            ]

    for j_max, a_max, v_max, p_start, p_end in tests:
        trajectory = test_fit_seven_segment(p_start, p_end, j_max, a_max, v_max)
    plot_trajectory(trajectory, j_max=j_max, a_max=a_max, v_max=v_max)

    # Plot the last one
    test_fit_seven_segment(p_start, p_end, j_max, a_max, v_max, True)
