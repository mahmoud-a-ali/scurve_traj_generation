import numpy as np
from sympy.core.numbers import Float

from .piecewise_function import PiecewiseFunction

# Turns on extra velidation of generated trajectories. The validation is quite expensive, so you probably want it off
# most of the time. Enabling it while debugging catches problems earlier, making it easier to find the bug.
VALIDATION_ENABLED = False


class InvalidTrajectory(Exception):
    pass


def validate(p_start, p_end, v_start, v_end, v_max, a_max, j_max, piecewise_jerk_function, independent_variable):
    piecewise_acceleration_function = piecewise_jerk_function.integrate(0.0)
    piecewise_velocity_function = piecewise_acceleration_function.integrate(v_start)
    piecewise_position_function = piecewise_velocity_function.integrate(p_start)
    boundaries = piecewise_jerk_function.boundaries

    if boundaries[0] != 0.0:
        raise InvalidTrajectory()

    # Boundary array must be sorted.
    if (boundaries[:-1] > boundaries[1:]).any():
        raise InvalidTrajectory()

    if not np.isclose(piecewise_velocity_function(boundaries[0]), v_start):
        raise InvalidTrajectory()

    if not np.isclose(piecewise_velocity_function(boundaries[-1]), v_end):
        raise InvalidTrajectory()

    if not np.isclose(piecewise_position_function(boundaries[0]), p_start):
        raise InvalidTrajectory()

    if not np.isclose(piecewise_position_function(boundaries[-1]), p_end):
        raise InvalidTrajectory()

    for independent_variable_value in boundaries:
        jerk = piecewise_jerk_function(independent_variable_value)
        acceleration = piecewise_acceleration_function(independent_variable_value)
        velocity = piecewise_velocity_function(independent_variable_value)
        position = piecewise_position_function(independent_variable_value)
        if np.abs(jerk) - np.abs(j_max) > 1e-8:
            raise InvalidTrajectory()
        if (np.abs(acceleration) - np.abs(a_max)) > 1e-8:
            raise InvalidTrajectory()
        if (np.abs(velocity) - np.abs(v_max)) > 1e-8:
            raise InvalidTrajectory()
        # if (np.abs(position) - np.abs(p_end)) > 1e-8:
        #    raise InvalidTrajectory()
    return True


def validate_acceleration_segments(independent_variable_start, v_start, v_end, a_max, j_max, piecewise_jerk_function):
    piecewise_acceleration_function = piecewise_jerk_function.integrate(0.0)
    piecewise_velocity_function = piecewise_acceleration_function.integrate(v_start)
    boundaries = piecewise_jerk_function.boundaries

    if not np.isfinite(boundaries).all():
        raise InvalidTrajectory()

    if boundaries[0] != independent_variable_start:
        raise InvalidTrajectory()

    # Boundary array must be sorted.
    if (boundaries[:-1] > boundaries[1:]).any():
        raise InvalidTrajectory()

    if not np.isclose(piecewise_velocity_function(boundaries[0]), v_start):
        raise InvalidTrajectory()

    if not np.isclose(piecewise_velocity_function(boundaries[-1]), v_end):
        raise InvalidTrajectory()

    for independent_variable_value in boundaries:
        jerk = piecewise_jerk_function(independent_variable_value)
        acceleration = piecewise_acceleration_function(independent_variable_value)
        if np.abs(jerk) > j_max:
            raise InvalidTrajectory()
        if (np.abs(acceleration) - np.abs(a_max)) > 1e-8:
            raise InvalidTrajectory()
        if not np.isfinite(jerk):
            raise InvalidTrajectory()
        if not np.isfinite(acceleration):
            raise InvalidTrajectory()
    return True


def fit_acceleration_triangle(v_start, v_end, a_max, j_max, independent_variable):
    """
    Positive acceleration triangle: v_end is greater than v_start, and the acceleration
    at the start and end is the same.
    """
    assert (a_max >= 0.0)
    assert (j_max >= 0.0)
    if np.isclose(v_start, v_end):
        return PiecewiseFunction([0.0, 0.0], [Float(0.0)], independent_variable)

    segment_duration = np.sqrt(np.abs((v_end - v_start) / j_max))
    j = np.sign(v_end - v_start) * j_max
    piecewise_jerk_function = PiecewiseFunction([0.0, segment_duration, 2.0 * segment_duration], [Float(j), Float(-j)],
                                                independent_variable)
    if VALIDATION_ENABLED:
        validate_acceleration_segments(0.0, v_start, v_end, a_max, j_max, piecewise_jerk_function)
    return piecewise_jerk_function


def fit_acceleration_trapezoid(v_start, v_end, a_max, j_max, independent_variable):
    assert (a_max >= 0.0)
    assert (j_max >= 0.0)
    if np.isclose(v_start, v_end):
        return PiecewiseFunction([0.0, 0.0], [Float(0.0)], independent_variable)

    j = np.sign(v_end - v_start) * j_max
    a = np.sign(v_end - v_start) * a_max

    ramp_duration = a_max / j_max
    ramp_velocity_change = 0.5 * a * ramp_duration
    segment_2_duration = np.abs(v_end - v_start - 2.0 * ramp_velocity_change) / a_max
    piecewise_jerk_function = PiecewiseFunction(
        np.cumsum([0.0, ramp_duration, segment_2_duration, ramp_duration]), [Float(j), Float(0.0), Float(-j)],
        independent_variable)

    if VALIDATION_ENABLED:
        validate_acceleration_segments(0.0, v_start, v_end, a_max, j_max, piecewise_jerk_function)
    return piecewise_jerk_function


def fit_acceleration_segments(v_start, v_end, a_max, j_max, independent_variable):
    """
    3 segment trajectory segment with positive max jerk, 0 jerk, and negative max jerk segments. The
    middle section may have zero duration if we can't reach max accleration. v_end is greater than v_start,
    and the acceleration at the start and end is zero.
    """
    min_velocity_change_if_reach_a_max = j_max * (a_max / j_max) ** 2.0
    if np.abs(min_velocity_change_if_reach_a_max) > np.abs(v_end - v_start):
        return fit_acceleration_triangle(v_start, v_end, a_max, j_max, independent_variable)
    else:
        return fit_acceleration_trapezoid(v_start, v_end, a_max, j_max, independent_variable)


def fit_given_cruising_velocity(p_start, p_end, v_start, v_end, v_cruise, a_max, j_max, independent_variable):
    jerk_for_first_three_segments = fit_acceleration_segments(v_start, v_cruise, a_max, j_max, independent_variable)
    if jerk_for_first_three_segments is None:
        return None

    jerk_for_last_three_segments = fit_acceleration_segments(v_cruise, v_end, a_max, j_max, independent_variable)
    if jerk_for_last_three_segments is None:
        return None

    acceleration_for_first_three_segments = jerk_for_first_three_segments.integrate(0.0)
    velocity_for_first_three_segments = acceleration_for_first_three_segments.integrate(v_start)
    position_for_first_three_segments = velocity_for_first_three_segments.integrate(p_start)
    segments_123_distance_traveled = position_for_first_three_segments(
        position_for_first_three_segments.boundaries[-1]) - position_for_first_three_segments(
        position_for_first_three_segments.boundaries[0])

    acceleration_for_last_three_segments = jerk_for_last_three_segments.integrate(0.0)
    velocity_for_last_three_segments = acceleration_for_last_three_segments.integrate(v_cruise)
    # We're not yet sure what the start position will be - it will depend on how long we maintain our cruising
    # velocity. For now we set it to 0.0 just to figure out how far we move in the last three segements.
    position_for_last_three_segments = velocity_for_last_three_segments.integrate(0.0)
    segments_567_distance_traveled = position_for_last_three_segments(
        position_for_last_three_segments.boundaries[-1]) - position_for_last_three_segments(
        position_for_last_three_segments.boundaries[0])

    segment_4_distance = (p_end - p_start - segments_123_distance_traveled - segments_567_distance_traveled)
    total_distance = segments_123_distance_traveled + segment_4_distance + segments_567_distance_traveled
    if np.abs(total_distance - (p_end - p_start)) > 1e-8:
        return None
    segment_4_duration = segment_4_distance / v_cruise
    if segment_4_duration < 0.0:
        return None

    jerk_for_segment_4 = PiecewiseFunction([0.0, segment_4_duration], [Float(0.0)], independent_variable)

    jerk = jerk_for_first_three_segments
    jerk.extend(jerk_for_segment_4)
    jerk.extend(jerk_for_last_three_segments)

    v_max = max(v_start, v_end, v_cruise)
    if VALIDATION_ENABLED:
        validate(p_start, p_end, v_start, v_end, v_max, a_max, j_max, jerk, independent_variable)
    return jerk


def fit(p_start, p_end, v_start, v_end, v_max, a_max, j_max, independent_variable,
        num_velocities_to_try=16):
    best_jerk = None
    best_end_time = np.inf
    for v_cruise in np.linspace(-v_max, v_max, num_velocities_to_try):
        jerk = fit_given_cruising_velocity(p_start, p_end, v_start, v_end, v_cruise, a_max, j_max, independent_variable)
        if jerk is None:
            continue

        end_time = jerk.boundaries[-1]
        if end_time < best_end_time:
            best_end_time = end_time
            best_jerk = jerk
    return best_jerk
