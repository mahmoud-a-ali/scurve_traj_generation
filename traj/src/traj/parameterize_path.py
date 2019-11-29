from sympy import Matrix, Piecewise, Symbol

def parameterize_path(path):
    """
    Represent the given joint-space path as a function q = f(s).
    """
    s = Symbol('s')
    boundaries = [0.0]
    functions = []
    for q0, q1 in zip(path[:-1], path[1:]):
        q0 = Matrix(q0)
        q1 = Matrix(q1)
        s0 = boundaries[-1]
        length = (q1 - q0).norm()
        s1 = s0 + length
        direction = (q1 - q0) / length
        boundaries.append(float(s1))
        functions.append(q0 + direction * (s - s0))
    return boundaries, functions

