#!/usr/bin/env python3

import os
import sys
import unittest

SCRIPT_DIR = os.path.join(os.path.dirname(__file__), '..', 'scripts')
sys.path.insert(0, os.path.abspath(SCRIPT_DIR))

from multi_bottle_evaluation_utils import (  # noqa: E402
    acceptance_passed,
    apply_local_offset,
    is_valid_position,
    minimum_cost_assignment,
    output_count_category,
    position_in_reference_frame,
    summarize_errors,
)


class MultiBottleEvaluationUtilsTest(unittest.TestCase):
    def test_rejects_invalid_positions(self):
        self.assertFalse(is_valid_position((0.0, 0.0, 0.0)))
        self.assertFalse(is_valid_position((1.0, float('nan'), 0.1)))
        self.assertTrue(is_valid_position((1.0, 0.0, 0.1)))

    def test_assignment_is_one_to_one_when_object_order_changes(self):
        truths = [(1.8, -0.45, 0.1), (2.5, 0.45, 0.1)]
        estimates = [(2.48, 0.45, 0.12), (1.78, -0.44, 0.12)]
        assignment = minimum_cost_assignment(truths, estimates)
        self.assertEqual([1, 0], [item[0] for item in assignment])
        self.assertLess(assignment[0][1], 0.04)
        self.assertLess(assignment[1][1], 0.04)

    def test_assignment_handles_three_targets(self):
        truths = [(1.65, -0.45, 0.1), (2.20, 0.0, 0.1), (2.65, 0.45, 0.1)]
        estimates = [(2.64, 0.46, 0.11), (1.66, -0.44, 0.11), (2.19, -0.01, 0.11)]
        assignment = minimum_cost_assignment(truths, estimates)
        self.assertEqual([1, 2, 0], [item[0] for item in assignment])
        for _, error in assignment:
            self.assertLess(error, 0.03)

    def test_assignment_handles_four_targets_for_distance_sweep(self):
        truths = [
            (1.65, -0.55, 0.1),
            (2.15, -0.20, 0.1),
            (2.60, 0.20, 0.1),
            (3.05, 0.55, 0.1),
        ]
        estimates = [
            (2.61, 0.19, 0.11),
            (3.04, 0.56, 0.11),
            (1.66, -0.54, 0.11),
            (2.14, -0.21, 0.11),
        ]
        assignment = minimum_cost_assignment(truths, estimates)
        self.assertEqual([2, 3, 0, 1], [item[0] for item in assignment])
        for _, error in assignment:
            self.assertLess(error, 0.03)

    def test_assignment_handles_single_target_for_far_recall(self):
        truths = [(2.5, 0.0, 0.1)]
        estimates = [(2.48, 0.01, 0.12)]
        assignment = minimum_cost_assignment(truths, estimates)
        self.assertEqual([0], [item[0] for item in assignment])
        self.assertLess(assignment[0][1], 0.04)


    def test_acceptance_keeps_default_ratio_thresholds_backward_compatible(self):
        per_model = {'ground_red_bottle': {'count': 10, 'median_m': 0.03}}
        self.assertTrue(acceptance_passed(
            timed_out=False, complete_output_messages=10, sample_count=10,
            invalid_output_messages=0, overcomplete_output_messages=0,
            per_model=per_model, maximum_median_error_m=0.05))

    def test_acceptance_can_require_minimum_complete_ratios(self):
        per_model = {'ground_red_bottle': {'count': 10, 'median_m': 0.03}}
        self.assertTrue(acceptance_passed(
            timed_out=False, complete_output_messages=10, sample_count=10,
            invalid_output_messages=0, overcomplete_output_messages=0,
            per_model=per_model, maximum_median_error_m=0.05,
            detection_complete_ratio=0.92, minimum_detection_complete_ratio=0.90,
            localization_complete_ratio=0.91,
            minimum_localization_complete_ratio=0.90))
        self.assertFalse(acceptance_passed(
            timed_out=False, complete_output_messages=10, sample_count=10,
            invalid_output_messages=0, overcomplete_output_messages=0,
            per_model=per_model, maximum_median_error_m=0.05,
            detection_complete_ratio=0.89, minimum_detection_complete_ratio=0.90,
            localization_complete_ratio=0.91,
            minimum_localization_complete_ratio=0.90))
        self.assertFalse(acceptance_passed(
            timed_out=False, complete_output_messages=10, sample_count=10,
            invalid_output_messages=0, overcomplete_output_messages=0,
            per_model=per_model, maximum_median_error_m=0.05,
            detection_complete_ratio=0.92, minimum_detection_complete_ratio=0.90,
            localization_complete_ratio=0.89,
            minimum_localization_complete_ratio=0.90))

    def test_summarizes_error_distribution(self):
        summary = summarize_errors([0.01, 0.02, 0.03, 0.04])
        self.assertEqual(4, summary['count'])
        self.assertAlmostEqual(0.025, summary['median_m'])
        self.assertAlmostEqual(0.0385, summary['p95_m'])
        self.assertEqual(0.01, summary['min_m'])
        self.assertEqual(0.04, summary['max_m'])

    def test_classifies_output_count_relative_to_models(self):
        self.assertEqual('undercomplete', output_count_category(1, 2))
        self.assertEqual('complete', output_count_category(2, 2))
        self.assertEqual('overcomplete', output_count_category(3, 2))

    def test_transforms_world_truth_through_unique_robot_model_pose(self):
        half_turn_z = (0.0, 0.0, 2 ** -0.5, 2 ** -0.5)
        center_world = apply_local_offset(
            (3.0, 1.0, 0.0), half_turn_z, (0.0, 0.0, 0.1))
        local = position_in_reference_frame(
            center_world, (1.0, 1.0, 0.0), half_turn_z)
        self.assertAlmostEqual(0.0, local[0], places=6)
        self.assertAlmostEqual(-2.0, local[1], places=6)
        self.assertAlmostEqual(0.1, local[2], places=6)

    def test_rejects_zero_quaternion(self):
        with self.assertRaises(ValueError):
            apply_local_offset((1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 0.0),
                               (0.0, 0.0, 0.1))


if __name__ == '__main__':
    unittest.main()
