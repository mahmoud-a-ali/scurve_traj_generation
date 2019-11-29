import numpy as np
from sympy import Matrix, Piecewise, Symbol

class PiecewiseFunction:
    def __init__(self, times, functions, independent_variable):
        self.times = times
        self.functions = functions
        self.independent_variable = independent_variable
        assert len(times) - 1 == len(functions)

    def __call__(self, value):
        if value == boundaries[-1]:
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
        return S, path_points

