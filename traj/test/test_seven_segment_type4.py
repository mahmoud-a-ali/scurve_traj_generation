import nose
import numpy as np
from sympy import Symbol

import traj


def check_fit_three_segment(independent_variable_start, v_start, v_end, a_max, j_max):
    t = Symbol('t')
    jerk = traj.seven_segment_type4.fit_three_segment(independent_variable_start, v_start, v_end, a_max, j_max, t, 8)
    acceleration = jerk.integral(0.0)
    velocity = acceleration.integral(0.0)

    t_start = velocity.boundaries[0]
    t_end = velocity.boundaries[-1]
    v_start_computed = velocity(t_start)
    v_end_computed = velocity(t_end)

    assert np.isclose(v_start, v_start_computed)
    assert np.isclose(v_end, v_end_computed)


def check_fit_three_segment_given_cruising_acceleration(independent_variable_start, v_start, v_end, a_cruise, j_max):
    t = Symbol('t')
    jerk = traj.seven_segment_type4.fit_three_segment_given_cruising_acceleration(independent_variable_start, v_start,
                                                                                  v_end, a_cruise, j_max,
                                                                                  t)
    acceleration = jerk.integral(0.0)
    velocity = acceleration.integral(v_start)

    t_start = velocity.boundaries[0]
    t_end = velocity.boundaries[-1]
    v_start_computed = velocity(t_start)
    v_end_computed = velocity(t_end)

    assert np.isclose(v_start, v_start_computed)
    assert np.isclose(v_end, v_end_computed)


def test_three_segment_given_cruising_acceleration():
    t = Symbol('t')
    # Postive acceleration, positive change in vleocity; relatively simple case.
    check_fit_three_segment_given_cruising_acceleration(0.0, 0.0, 1.0, 1.0, 10.0)

    # Keep the same velocity - all segments should have zero length.
    check_fit_three_segment_given_cruising_acceleration(0.0, 0.0, 0.0, 0.0, 10.0)

    # Can't achieve positive change in velocity using negative acceleration.
    assert traj.seven_segment_type4.fit_three_segment_given_cruising_acceleration(
        0.0, 0.0, -1.0, 1.0, 10.0, t) is None


def test_seven_segment_given_cruising_acceleration():
    t = Symbol('t')
    j_max = 0.1

    jerk = traj.seven_segment_type4.fit_given_cruising_velocity(0.0, 30.0, 0.0, 0.0, 5.0, 2.0, 0.1, t)
    assert jerk is not None


nose.main()
