#!/usr/bin/env python3
import os
import sys
import unittest

SCRIPT_DIR = os.path.join(os.path.dirname(__file__), '..', 'scripts')
sys.path.insert(0, os.path.abspath(SCRIPT_DIR))

import run_random_three_garbage_competition_regression as random_runner


class RandomThreeGarbageRegressionTest(unittest.TestCase):
    def test_same_seed_reproduces_case(self):
        import random
        first = random_runner.choose_case(random.Random(42))
        second = random_runner.choose_case(random.Random(42))
        self.assertEqual(first, second)

    def test_case_seed_reproduces_selected_rooms(self):
        seed = 123456789
        first = random_runner.choose_case_from_seed(seed)
        second = random_runner.choose_case_from_seed(seed)
        self.assertEqual(first['rooms'], second['rooms'])
        self.assertEqual(first['enabled'], second['enabled'])

    def test_selected_rooms_are_three_distinct_valid_rooms(self):
        import random
        case = random_runner.choose_case(random.Random(7))
        self.assertEqual(3, len(case['rooms']))
        self.assertEqual(3, len(set(case['rooms'])))
        self.assertTrue(set(case['rooms']).issubset(set(random_runner.SLOTS)))
        self.assertEqual(3, sum(case['enabled'].values()))

    def test_trash_bin_is_not_randomized(self):
        import random
        case = random_runner.choose_case(random.Random(8))
        self.assertTrue(case['trash_bin_fixed'])
        self.assertNotIn('trash_bin', case['targets'])

    def test_scene_command_disables_unselected_slots(self):
        import random
        args = random_runner.parse_args(['--cases', '1'])
        case = random_runner.choose_case(random.Random(3))
        command = random_runner.scene_launch_command(args, case)
        for room, (arg_name, _class_name) in random_runner.SLOTS.items():
            expected = '{}:={}'.format(arg_name, str(case['enabled'][room]).lower())
            self.assertIn(expected, command)

    def test_cases_must_be_positive(self):
        with self.assertRaises(SystemExit):
            random_runner.parse_args(['--cases', '0'])


if __name__ == '__main__':
    unittest.main()
