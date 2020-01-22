import numpy as np


class PiecewiseFunction:
    """
    A piecewise function of a single variable.

    Uses sympy as its function representation, but because we know we are a function of
    one variable, we are able to compute integrals correctly (unlike sympy's builtin piecewise
    function).
    """

    def __init__(self, boundaries, functions, independent_variable):
        self.boundaries = np.asarray(boundaries)
        self.functions = functions
        self.independent_variable = independent_variable
        assert len(boundaries) - 1 == len(functions)

    def __call__(self, value):
        if value == self.boundaries[-1]:
            # For convenience, we include the final time in the last segment
            func_i = len(self.functions) - 1
        else:
            func_i = np.searchsorted(self.boundaries, value, side='right') - 1
        value_relative = value - self.boundaries[func_i]
        return np.array(self.functions[func_i].subs(
            self.independent_variable, value_relative)).astype(np.float64).flatten()

    def extend(self, other):
        # If the start of the other piecewise function isn't zero, there would be a gap in the middle of the
        # combined function that would have no defined values. We don't have any good way to handle that.
        assert (other.boundaries[0] == 0.0)
        self.boundaries = np.concatenate((self.boundaries[:-1], other.boundaries + self.boundaries[-1]))
        self.functions = np.concatenate((self.functions, other.functions))

    def sample(self, npoints):
        independent_variable_values = np.linspace(
            self.boundaries[0], self.boundaries[-1], npoints)
        path_points = np.array([self(v) for v in independent_variable_values])
        return independent_variable_values, path_points

    def integrate(self, integration_constant):
        integrated_functions = [integration_constant + self.functions[0].integrate(self.independent_variable)]
        for function_i in range(1, len(self.functions)):
            start_value = integrated_functions[function_i - 1].subs(self.independent_variable,
                                                                    self.boundaries[function_i] - self.boundaries[
                                                                        function_i - 1])
            integrated_functions.append(start_value + self.functions[function_i].integrate(self.independent_variable))
        return PiecewiseFunction(self.boundaries[:], integrated_functions, self.independent_variable)
