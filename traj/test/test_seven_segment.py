import nose
import numpy as np

from traj import fit_seven_segment

def check_fit_seven_segment(p_start, p_end, v_max, a_max, j_max):
    position, velocity, jerk, acceleration = fit_seven_segment(
            p_start, p_end, v_max, a_max, j_max)

    t_start = position.boundaries[0]
    t_end = position.boundaries[-1]
    p_start_computed = position(t_start)
    v_start_computed = velocity(t_start)
    p_end_computed = position(t_end)[0]
    v_end_computed = velocity(t_end)[0]

    assert np.isclose(p_start, p_start_computed)
    assert np.isclose(p_end, p_end_computed)

def test_max_vel_but_not_max_acc():
    check_fit_seven_segment(0.0, 30.0, 0.1, 3.0, 0.1)

def test_max_acc_but_not_max_vel():
    check_fit_seven_segment(0.0, 30.0, 6.0, 3.0, 0.1)

def test_not_max_vel_or__max_acc():
    check_fit_seven_segment(0.0, 0.4, 3.1, 2.3, 0.1)

def test_max_vel_and_max_acc():
    check_fit_seven_segment(0.0, 30.0, 2.0, 0.4, 0.1)

