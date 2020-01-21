from sympy import integrate
from sympy.core.numbers import Float

from .piecewise_function import PiecewiseFunction


def fit(p_start, p_end, v_max, a_max, j_max, independent_variable):
    """
    Find the optimal seven segment trajectory for zero start and end velocities, and the given
    start and end positions.

    Follows the nomenclature and approach of

        Herrera-Aguilar, Ignacio, and Daniel Sidobre. "Soft motion trajectory planning and
        control for service manipulator robot." Workshop on Physical Human-Robot Interaction in
        Anthropic Domains at IROS. 2006.
    """
    assert (a_max > 0.0)
    assert (j_max > 0.0)
    assert (v_max > 0.0)

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
        T_jmax = (v_max / j_max) ** 0.5
        print(T_jmax, j_max)
        a_max = T_jmax * j_max

    # Compute the minimum distance that each case can travel. D_thr1 is the minimum distance for a
    # trajectory that hits both max acceleration and max velocity. D_thr2 is the minimum distance
    # for a trajectory that hits max acceleration but not max velocity.
    D_thr1 = (a_max * v_max) / j_max + v_max ** 2 / a_max
    D_thr2 = 2.0 * a_max ** 3 / j_max ** 2

    D = p_end - p_start
    if D >= D_thr1:
        # We hit both v_max and a_max
        T_j = T_jmax
        T_a = T_amax
        T_v = (D - D_thr1) / v_max
    elif D > D_thr2:
        # We hit a_max but not v_max
        T_v = 0.0
        T_j = T_jmax
        T_a = (a_max ** 2 / (4.0 * j_max) + D / a_max) ** 0.5 - 1.5 * a_max / j_max
    else:
        # We hit neither a_max nor v_max
        T_v = 0.0
        T_a = 0.0
        T_j = (D / (2.0 * j_max)) ** (1.0 / 3.0)

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
    # Integrate jerk starting from the start of the trajectory and going all the way through the end.
    for j0, T in segment_jerks_and_durations:
        times.append(times[-1] + T)
        jerk_functions.append(Float(j0))

    jerk = PiecewiseFunction(times, jerk_functions, independent_variable)
    return jerk
