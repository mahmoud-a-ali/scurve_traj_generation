import numpy as np
from sympy.core.numbers import Float

from .piecewise_function import PiecewiseFunction


class InvalidTrajectory(Exception):
    pass


def validate(p_start, p_end, v_start, v_end, v_max, a_max, j_max, piecewise_jerk_function, independent_variable):
    piecewise_acceleration_function = piecewise_jerk_function.integral(0.0)
    piecewise_velocity_function = piecewise_acceleration_function.integral(v_start)
    piecewise_position_function = piecewise_velocity_function.integral(p_start)
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
        #if (np.abs(position) - np.abs(p_end)) > 1e-8:
        #    raise InvalidTrajectory()
    return True


def validate_three_segment(independent_variable_start, v_start, v_end, a_max, j_max, piecewise_jerk_function):
    piecewise_acceleration_function = piecewise_jerk_function.integral(0.0)
    piecewise_velocity_function = piecewise_acceleration_function.integral(v_start)
    boundaries = piecewise_jerk_function.boundaries

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
    return True


def fit_three_segment_given_cruising_acceleration(independent_variable_start, v_start, v_end, a_cruise, j_max,
                                                  independent_variable):
    segment_1_jerk = np.sign(a_cruise) * j_max
    segment_2_jerk = 0.0
    segment_3_jerk = np.sign(-a_cruise) * j_max
    if a_cruise == 0.0:
        # We special case this "stay in one place" condition because we'd run into
        # divide-by-zero problems if we tried to calculate out the durations of
        # each segment.
        if v_end - v_start == 0.0:
            segment_1_duration = 0.0
            segment_2_duration = 0.0
            segment_3_duration = segment_1_duration
        else:
            return None
    else:
        segment_1_duration = a_cruise / segment_1_jerk
        assert (segment_1_duration >= 0.0)
        segment_3_duration = segment_1_duration
        v_start_of_segment_2 = v_start + 0.5 * segment_1_jerk * segment_1_duration ** 2.0
        v_end_of_segment_2 = v_end + 0.5 * segment_3_jerk * segment_3_duration ** 2.0
        segment_2_duration = (v_end_of_segment_2 - v_start_of_segment_2) / a_cruise
        if segment_2_duration < 0.0:
            return None

        if not np.isfinite(segment_1_duration) or not np.isfinite(segment_2_duration):
            return None

    boundaries = [independent_variable_start]
    for duration in [segment_1_duration, segment_2_duration, segment_3_duration]:
        boundaries.append(boundaries[-1] + duration)

    jerk_functions = np.array([Float(segment_1_jerk), Float(segment_2_jerk), Float(segment_3_jerk)])
    piecewise_jerk_function = PiecewiseFunction(boundaries, jerk_functions, independent_variable)

    validate_three_segment(independent_variable_start, v_start, v_end, a_cruise, j_max, piecewise_jerk_function)
    return piecewise_jerk_function


def fit_three_segment(independent_variable_start, v_start, v_end, a_max, j_max, independent_variable,
                      num_accelerations_to_try=16):
    best_piecewise_jerk = None
    candidate_cruise_accelerations = np.sign(v_end - v_start) * np.linspace(a_max / float(num_accelerations_to_try),
                                                                            a_max, num_accelerations_to_try)
    for a_cruise in candidate_cruise_accelerations:
        piecewise_jerk = fit_three_segment_given_cruising_acceleration(independent_variable_start, v_start, v_end,
                                                                       a_cruise, j_max,
                                                                       independent_variable)
        if piecewise_jerk is None:
            # Higher cruising accelerations definitely aren't feasible, return the best trajectory we've found.
            return best_piecewise_jerk
        best_piecewise_jerk = piecewise_jerk
    return best_piecewise_jerk

def fit_given_cruising_velocity(p_start, p_end, v_start, v_end, v_cruise, a_max, j_max, independent_variable):
    jerk_for_first_three_segments = fit_three_segment(0.0, v_start, v_cruise, a_max, j_max, independent_variable)
    if jerk_for_first_three_segments is None:
        return None

    jerk_for_last_three_segments = fit_three_segment(0.0, v_cruise, v_end, a_max, j_max,
                                                     independent_variable)
    if jerk_for_last_three_segments is None:
        return None

    acceleration_for_first_three_segments = jerk_for_first_three_segments.integral(0.0)
    velocity_for_first_three_segments = acceleration_for_first_three_segments.integral(v_start)
    position_for_first_three_segments = velocity_for_first_three_segments.integral(p_start)
    segments_123_distance_traveled = position_for_first_three_segments(
        position_for_first_three_segments.boundaries[-1]) - position_for_first_three_segments(
        position_for_first_three_segments.boundaries[0])

    acceleration_for_last_three_segments = jerk_for_last_three_segments.integral(0.0)
    velocity_for_last_three_segments = acceleration_for_last_three_segments.integral(v_cruise)
    # We're not yet sure what the start position will be - it will depend on how long we maintain our cruising
    # velocity. For now we set it to 0.0 just to figure out how far we move in the last three segements.
    position_for_last_three_segments = velocity_for_last_three_segments.integral(0.0)
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
