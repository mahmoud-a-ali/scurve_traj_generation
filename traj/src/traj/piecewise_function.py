import numpy as np
from sympy import Matrix, Piecewise, Symbol

class PiecewiseFunction:
    """
    A piecewise function of a single variable.

    Uses sympy as its function representation, but because we know we are a function of
    one variable, we are able to compute integrals correctly (unlike sympy's builtin piecewise
    function).
    """
    def __init__(self, boundaries, functions, independent_variable):
        self.boundaries = boundaries
        self.functions = functions
        self.independent_variable = independent_variable
        assert len(boundaries) - 1 == len(functions)

    def __call__(self, value):
        if value == self.boundaries[-1]:
            # For convenience, we include the final time in the last segment
            func_i = len(self.functions) - 1
        else:
            func_i = np.searchsorted(self.boundaries, value, side='right') - 1
        return np.array(self.functions[func_i].subs(
            self.independent_variable, value)).astype(np.float64).flatten()

    def sample(self, npoints):
        independent_variable_values = np.linspace(
                self.boundaries[0], self.boundaries[-1], npoints)
        path_points = np.array([self(v) for v in independent_variable_values])
        return independent_variable_values, path_points

