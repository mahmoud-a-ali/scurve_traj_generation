from sympy import Float, Matrix, Piecewise, Symbol

from piecewise_function import PiecewiseFunction


def blend_ratio(s):
    """
    Carefully chosen polynomial such that:
      blend_ratio(0.0) is 0.0
      blend_ration(1.0) is 1.0)
      derivative of blend_ratio w.r.t s at s=0.0 is 0.0
      derivative of blend_ratio w.r.t. s at s=1.0 is 0.0
    """
    return 6. * (s ** 5) - 15 * (s ** 4) + 10 * (s ** 3)


def corrected_blend_ratio(s, s0, s1):
    """
    The blend_ratio() polynomial assumes that s is between 0.0 and 1.0. Here we scale s to fit those assumptions.

    Args:
        s - value of s for which we want to evaluate the blend ratio
        s0 - value of s at the beginning of the blended segment.
        s1 - value of s at the end of the blended segment.
    """
    return blend_ratio((s - s0) / (s1 - s0))


def create_blended_segment(segment1_function, segment2_function, blend_radius):
    pass


def parameterize_path(path):
    """
    Represent the given joint-space path as a function q = f(s).

    To make this useful as a first step to creating smooth trajectories, we consruct the
    parameterization such that it has a few important properties:

    1. s = 0 is the start point of the path in joint space.
    2. As s increases, we move along the path monotonically.
    3. The length of path traversed for a given increase in s is constant.

    Out of convenience, we choose a parameterization such that the value of s is equal
    to the length of path travesed. This "length" is in N-dimensional joint space, but
    otherwise is the same as we would compute a path length.

    Put another way, the path length from s=0 to s=S is equal to the integral from 0 to S
    of the norm of the derivative of the parameterized path function w.r.t. the variable s.
    """
    s = Symbol('s')
    boundaries = [0.0]
    functions = []
    # q0 and q1 are successive joint space positions in the path. "boundaries" are the values of the
    # independent variable (often time) at which we switch from one function to the next in our
    # piecewise representation.
    for q0, q1 in zip(path[:-1], path[1:]):
        q0 = Matrix(q0)
        q1 = Matrix(q1)
        s0 = boundaries[-1]
        length = (q1 - q0).norm()
        s1 = s0 + length
        direction = (q1 - q0) / length
        boundaries.append(float(s1))
        functions.append(q0 + direction * s)
    return PiecewiseFunction(boundaries, functions, s)


def blend_parameterized_path(piecewise_position_function, blend_radius):
    """
    blend_radius is in same units as the independent variable of the piecewise position function "s". In the
    un-blended path we start with, these units are equal to the distance in joint space.
    """
    s = piecewise_position_function.independent_variable

    boundaries = piecewise_position_function.boundaries
    functions = piecewise_position_function.functions
    assert len(boundaries) == len(functions) + 1
    blended_boundaries = [0.0]
    blended_functions = []
    for segment_i in range(len(functions)):
        original_segment_length = boundaries[segment_i + 1] - boundaries[segment_i]
        # Relative to start of the original segment
        if segment_i == 0:
            s_start = 0.0
            s_end = original_segment_length - blend_radius
        elif segment_i == len(functions) - 1:
            s_start = blend_radius
            s_end = original_segment_length
        else:
            s_start = blend_radius
            s_end = original_segment_length - blend_radius

        segment_length = s_end - s_start
        if segment_length < 0.0:
            segment_length = boundaries[segment_i + 1] - boundaries[segment_i]
            raise RuntimeError('Segment {} has length {} which is less than 2*blend_radius', segment_i, segment_length)

        blended_boundaries.append(blended_boundaries[-1] + segment_length)
        blended_functions.append(functions[segment_i].subs(s, s + s_start))

        # TODO: Add the blend portion

        blended_boundaries.append(blended_boundaries[-1] + blend_radius * 2.0)
        blended_functions.append(Matrix([Float(0.0), Float(0.0)]))

    return PiecewiseFunction(blended_boundaries, blended_functions, s)
