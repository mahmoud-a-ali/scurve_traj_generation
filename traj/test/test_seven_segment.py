import nose

from traj import fit_seven_segment

def check_fit_seven_segment(j_max, a_max, v_max, p_start, p_end):
    position, velocity, jerk, acceleration = fit_seven_segment(
            p_start, p_end, j_max, a_max, v_max)

    t_start = position.boundaries[0]
    t_end = position.boundaries[-1]
    p_start_computed = position(t_start)
    v_start_computed = velocity(t_start)
    p_end_computed = position(t_end)
    v_end_computed = velocity(t_end)

    nose.tools.eq_(p_start, p_start_computed)
    nose.tools.eq_(p_end, p_end_computed)

def test_max_vel_but_not_max_acc():
    check_fit_seven_segment(0.1, 3.0, 0.1, 0.0, 30.0)

def test_max_acc_but_not_max_vel():
    check_fit_seven_segment(0.1, 3.0, 6.0, 0.0, 30.0)

def test_not_max_vel_or__max_acc():
    check_fit_seven_segment(0.1, 2.3, 3.1, 0.0, 0.4)

def test_max_vel_and_max_acc():
    check_fit_seven_segment(0.1, 0.4, 2.0, 0.0, 30.0)

