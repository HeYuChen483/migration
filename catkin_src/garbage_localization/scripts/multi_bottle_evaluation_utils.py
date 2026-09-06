#!/usr/bin/env python3
"""Pure helpers for Gazebo multi-bottle localization acceptance."""

from __future__ import print_function

import itertools
import math


def is_valid_position(position):
    """Return True only for finite, non-zero XYZ tuples."""
    if len(position) != 3 or not all(math.isfinite(value) for value in position):
        return False
    return any(abs(value) > 1e-9 for value in position)


def quaternion_conjugate(quaternion):
    """Return the conjugate of an XYZW quaternion."""
    return (-quaternion[0], -quaternion[1], -quaternion[2], quaternion[3])


def rotate_vector(vector, quaternion):
    """Rotate an XYZ vector by a normalized XYZW quaternion."""
    x, y, z, w = quaternion
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12 or not math.isfinite(norm):
        raise ValueError('quaternion must be finite and non-zero')
    x, y, z, w = (x / norm, y / norm, z / norm, w / norm)
    vx, vy, vz = vector
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (vx + w * tx + y * tz - z * ty,
            vy + w * ty + z * tx - x * tz,
            vz + w * tz + x * ty - y * tx)


def position_in_reference_frame(world_position, reference_position,
                                reference_orientation):
    """Transform a world XYZ point into a reference pose's local frame."""
    relative = tuple(value - origin for value, origin
                     in zip(world_position, reference_position))
    return rotate_vector(relative, quaternion_conjugate(reference_orientation))


def apply_local_offset(position, orientation, local_offset):
    """Apply an object-local XYZ offset to a world pose position."""
    rotated_offset = rotate_vector(local_offset, orientation)
    return tuple(value + offset for value, offset
                 in zip(position, rotated_offset))


def output_count_category(actual_count, expected_count):
    """Classify an output count relative to the expected model count."""
    if actual_count < expected_count:
        return 'undercomplete'
    if actual_count > expected_count:
        return 'overcomplete'
    return 'complete'


def euclidean_distance(first, second):
    return math.sqrt(sum((left - right) ** 2 for left, right in zip(first, second)))


def minimum_cost_assignment(truth_positions, estimated_positions):
    """Match equally sized point sets without allowing repeated assignments.

    The returned list is ordered by truth_positions and contains estimated indices
    and their Euclidean errors.  None means that no valid one-to-one match exists.
    """
    if not truth_positions or len(truth_positions) != len(estimated_positions):
        return None
    if (not all(is_valid_position(point) for point in truth_positions) or
            not all(is_valid_position(point) for point in estimated_positions)):
        return None

    best = None
    for permutation in itertools.permutations(range(len(estimated_positions))):
        errors = [euclidean_distance(truth_positions[index],
                                     estimated_positions[estimated_index])
                  for index, estimated_index in enumerate(permutation)]
        candidate = (sum(errors), permutation, errors)
        if best is None or candidate[0] < best[0]:
            best = candidate

    return [(estimated_index, error)
            for estimated_index, error in zip(best[1], best[2])]


def percentile(values, fraction):
    """Return a linearly interpolated percentile for a non-empty list."""
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    index = (len(ordered) - 1) * fraction
    lower = int(math.floor(index))
    upper = int(math.ceil(index))
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (index - lower)


def summarize_errors(values):
    """Produce meters-based summary values suitable for JSON output."""
    if not values:
        return {'count': 0, 'median_m': None, 'p95_m': None,
                'min_m': None, 'max_m': None}
    return {
        'count': len(values),
        'median_m': percentile(values, 0.5),
        'p95_m': percentile(values, 0.95),
        'min_m': min(values),
        'max_m': max(values),
    }


def acceptance_passed(timed_out, complete_output_messages, sample_count,
                      invalid_output_messages, overcomplete_output_messages,
                      per_model, maximum_median_error_m,
                      detection_complete_ratio=0.0,
                      minimum_detection_complete_ratio=0.0,
                      localization_complete_ratio=0.0,
                      minimum_localization_complete_ratio=0.0):
    """Return True when count, error, validity, and recall criteria pass."""
    return (not timed_out and
            complete_output_messages >= sample_count and
            invalid_output_messages == 0 and
            overcomplete_output_messages == 0 and
            detection_complete_ratio >= minimum_detection_complete_ratio and
            localization_complete_ratio >= minimum_localization_complete_ratio and
            all(summary['count'] == sample_count and
                summary['median_m'] <= maximum_median_error_m
                for summary in per_model.values()))
