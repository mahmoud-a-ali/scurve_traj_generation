import nose
import numpy as np
from sympy import diff, Symbol

import traj


def test_blend_function():
    s = Symbol('s')
    # To create a blend that connects to both the previous and next segment, the value must be 0.0 for s=0.0,
    # 1.0 for s=1.0. For the connection between the blended portion and the neighboring segments to be smooth, the
    # derivative of the blend ratio must be 0 at both ends.
    blend_ratio_function = traj.blend_ratio(s)
    assert blend_ratio_function.subs(s, 0.0) == 0.0
    assert diff(blend_ratio_function, s).subs(s, 0.0) == 0.0
    assert blend_ratio_function.subs(s, 1.0) == 1.0
    assert diff(blend_ratio_function, s).subs(s, 1.0) == 0.0

def test_corrected_blend_function():
    s = Symbol('s')
    corrected_blend_ratio_function = traj.corrected_blend_ratio(s, 0.0, 10.9)
    assert np.isclose(float(corrected_blend_ratio_function.subs(s, 0.0)), 0.0, 1e-7)
    assert np.isclose(float(corrected_blend_ratio_function.subs(s, 10.9)), 1.0, 1e-7)
