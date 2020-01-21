import nose
import numpy as np
from sympy import Symbol

import traj

traj.VALIDATION_ENABLED = True


def check_fit_acceleration_segments(v_start, v_end, a_max, j_max):
    t = Symbol('t')
    jerk = traj.seven_segment_type4.fit_acceleration_segments(v_start, v_end, a_max, j_max, t)
    acceleration = jerk.integrate(0.0)
    velocity = acceleration.integrate(v_start)

    t_start = velocity.boundaries[0]
    t_end = velocity.boundaries[-1]
    v_start_computed = velocity(t_start)
    v_end_computed = velocity(t_end)

    assert np.isclose(v_start, v_start_computed)
    assert np.isclose(v_end, v_end_computed)
    return jerk, acceleration, velocity


def check_fit(p_start, p_end, v_start, v_end, v_max, a_max, j_max):
    t = Symbol('t')
    jerk = traj.seven_segment_type4.fit(p_start, p_end, v_start, v_end, v_max, a_max, j_max, t)
    acceleration = jerk.integrate(0.0)
    velocity = acceleration.integrate(v_start)
    position = velocity.integrate(p_start)

    t_start = velocity.boundaries[0]
    t_end = velocity.boundaries[-1]
    v_start_computed = velocity(t_start)
    p_start_computed = velocity(0.0)
    v_end_computed = velocity(t_end)
    p_end_computed = position(t_end)

    assert np.isclose(v_start, v_start_computed)
    assert np.isclose(p_start, p_start_computed)
    assert np.isclose(v_end, v_end_computed)
    assert np.isclose(p_end, p_end_computed)
    return jerk, acceleration, velocity, position


def test_fit_acceleration_segments():
    t = Symbol('t')

    # Positive change in velocity; relatively simple case.
    check_fit_acceleration_segments(0.0, 1.0, 1.0, 10.0)

    # Non-zero starting velocity.
    check_fit_acceleration_segments(-0.67, 1.6, 1.0, 10.0)

    # Negative change in velocity.
    check_fit_acceleration_segments(0.0, -1.0, 1.0, 10.0)

    # Not enough time to reach max acceleration, so this should return an acceleration triangle with two segments.
    jerk, _, _ = check_fit_acceleration_segments(0.0, 1.0, 1e6, 10.0)
    assert len(jerk.functions) == 2

    # Keep the same velocity - should return one zero length segment.
    jerk, _, _ = check_fit_acceleration_segments(0.0, 0.0, 0.0, 10.0)
    assert len(jerk.functions) == 1


def test_seven_segment_given_cruising_acceleration():
    t = Symbol('t')

    check_fit(0.0, 30.0, 0.0, 0.0, 3.0, 2.0, 10.0)
