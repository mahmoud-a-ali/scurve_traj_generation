from sympy import integrate, Symbol
from sympy.core.numbers import Float

from .piecewise_function import PiecewiseFunction
import traj



def fit_traj_segment(p_start, p_end, v_start, v_end, p_max, v_max, a_max, j_max, independent_variable=Symbol('t')):
    """
    Fit a trajectory segment for given start and end velocities/positions!
    
    """
    assert(a_max > 0.0)
    assert(j_max > 0.0)
    assert(v_max > 0.0)

    if(v_start > v_max or v_end > v_max):
        print"\nWarning: \n>>> these values are not feasible,  v_start and v_end should be within the limit v_max !"
        exit()
    if(p_start > p_max or p_end > p_max):
        print"\nWarning: \n>>> these values are not feasible,  p_start and p_end should be within the limit p_max !"
        exit()
        
    t_jrk_to_vf, t_acc_to_vf, t_jrk, t_acc, t_vel = traj.traj_segment_planning(p_start, p_end, v_start, v_end, v_max, a_max, j_max)
#    print "t_jrk_to_vf, t_acc_to_vf, t_jrk, t_acc, t_vel: "
#    print t_jrk_to_vf, t_acc_to_vf, t_jrk, t_acc, t_vel
    
    if v_end != v_start:
         j_max_to_vf = ((v_end-v_start)/abs(v_end-v_start))*j_max
    else:
         j_max_to_vf=0
    
    if v_end > v_start:
        segment_jerks_and_durations = [(j_max_to_vf, t_jrk_to_vf), (0.0, t_acc_to_vf), (-j_max_to_vf, t_jrk_to_vf),    (j_max, t_jrk), (0.0, t_acc), (-j_max, t_jrk), (0.0, t_vel), (-j_max,t_jrk), (0.0, t_acc), (j_max, t_jrk)]
    else:
        segment_jerks_and_durations = [(j_max, t_jrk), (0.0, t_acc), (-j_max, t_jrk), (0.0, t_vel), (-j_max,t_jrk), (0.0, t_acc), (j_max, t_jrk),    (j_max_to_vf, t_jrk_to_vf), (0.0, t_acc_to_vf), (-j_max_to_vf, t_jrk_to_vf)]
    
#    segments = []
    p0 = p_start
    v0 = v_start
    a0 = 0.0
    times = [0.0]
    jerk_functions = []
    acceleration_functions = []
    velocity_functions = []
    position_functions = []
    # Integrate jerk starting from the start of the trajectory and going all the way through the end.
    for j0, T in segment_jerks_and_durations:
        times.append(times[-1] + T)
        j = Float(j0)
        a = integrate(j, independent_variable) + a0
        v = integrate(a, independent_variable) + v0
        p = integrate(v, independent_variable) + p0
        jerk_functions.append(j)
        acceleration_functions.append(a)
        velocity_functions.append(v)
        position_functions.append(p)
        a0 = a.subs({independent_variable: T})
        v0 = v.subs({independent_variable: T})
        p0 = p.subs({independent_variable: T})
    position = PiecewiseFunction(times, position_functions, independent_variable)
    velocity = PiecewiseFunction(times, velocity_functions, independent_variable)
    acceleration = PiecewiseFunction(times, acceleration_functions, independent_variable)
    jerk = PiecewiseFunction(times, jerk_functions, independent_variable)
    return position, velocity, acceleration, jerk
