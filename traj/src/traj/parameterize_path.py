import copy

import numpy as np
from sympy import Float, Matrix, Piecewise, Symbol, sin, cos

from piecewise_function import PiecewiseFunction

# Values smaller than this are considered to be zero to avoid numerical problems.
PRECISION = 1e-6


def create_arc_segment(q_blend_start, q_unblended_waypoint, q_blend_end, blend_radius, s):
    # Make sure all arguments are simple numpy arrays (not sympy matrices).
    q_blend_start = np.array(q_blend_start).astype(np.float64).flatten()
    q_unblended_waypoint = np.array(q_unblended_waypoint).astype(np.float64).flatten()
    q_blend_end = np.array(q_blend_end).astype(np.float64).flatten()

    v1 = q_blend_start - q_unblended_waypoint
    v2 = q_blend_end - q_unblended_waypoint
    v1 /= np.linalg.norm(v1)
    v2 /= np.linalg.norm(v2)

    # Included angle between the two segments
    included_dot_product = np.dot(v1, v2)
    if np.abs(included_dot_product) > 1.0 - PRECISION:
        return 0.0, Matrix(q_blend_start)

    theta = np.arccos(included_dot_product)

    # Distance from unblended waypoint to centerpoint of arc.
    L = blend_radius / np.cos(theta / 2.0)

    centerline_vector = (v1 + v2) / 2.0
    centerline_vector /= np.linalg.norm(centerline_vector)

    arc_centerpoint = q_unblended_waypoint + L * centerline_vector

    arc_radius = np.linalg.norm(q_blend_start - arc_centerpoint)

    alpha = np.pi - theta
    chord_vector = q_blend_end - q_blend_start
    chord_length = np.linalg.norm(chord_vector)
    chord_vector /= chord_length

    arc_length = arc_radius * alpha

    angle = (alpha / 2.0 - s / arc_radius)

    return arc_length, Matrix(arc_centerpoint) + arc_radius * Matrix(-centerline_vector) * cos(
        angle) + arc_radius * Matrix(-chord_vector) * sin(angle)


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


def parameterize_path_with_blends(path, blend_radius):
    # We modify the path in place when we add blends. To avoid changing the path which was passed
    # in, we make a copy here.
    path = copy.deepcopy(path[:])

    s = Symbol('s')
    boundaries = [0.0]
    functions = []
    # q0 and q1 are successive joint space positions in the path. "boundaries" are the values of the
    # independent variable (often time) at which we switch from one function to the next in our
    # piecewise representation.
    for point_i in range(len(path) - 1):
        q0 = Matrix(path[point_i])
        q1 = Matrix(path[point_i + 1])
        s0 = boundaries[-1]
        length = (q1 - q0).norm()
        s1 = s0 + length
        direction = (q1 - q0) / length

        if point_i == len(path) - 2:
            # Last point; no blend
            boundaries.append(float(s1))
            functions.append(q0 + direction * s)
        else:
            # This segments ends where it enters the blend sphere.
            segment_length = s1 - s0 - blend_radius
            boundaries.append(boundaries[-1] + segment_length)
            functions.append(q0 + direction * s)

            q_blend_start = functions[-1].subs(s, segment_length)

            q2 = Matrix(path[point_i + 2])
            length_next = (q2 - q1).norm()
            direction_next = (q2 - q1) / length_next
            q_blend_end = q1 + direction_next * blend_radius

            arc_length, arc_function = create_arc_segment(q_blend_start, q1, q_blend_end,
                                                          blend_radius, s)

            boundaries.append(boundaries[-1] + arc_length)
            functions.append(arc_function)

            path[point_i + 1] = np.array(functions[-1].subs(s, arc_length)).astype(
                np.float64).flatten()

    return PiecewiseFunction(boundaries, functions, s)
