#!/usr/bin/env python3
import math
import unittest

from room_global_localizer.core import choose_ndt_winner


class NdtSelectionTest(unittest.TestCase):
    def test_correlated_fallbacks_cannot_outvote_reliable_primary(self):
        candidates = [
            {'score': 0.680},
            {'score': 0.621},
            {'score': 0.610},
        ]
        reliable = [
            {'seed_index': 0, 'x': 24.052, 'y': -12.430,
             'yaw': math.radians(82.74)},
            {'seed_index': 1, 'x': 21.418, 'y': -8.322,
             'yaw': math.radians(-97.56)},
            {'seed_index': 2, 'x': 21.397, 'y': -8.316,
             'yaw': math.radians(-97.91)},
        ]

        winner, validation, support, cluster_count = choose_ndt_winner(
            reliable, candidates, 0.75, math.radians(10.0))

        self.assertEqual(winner['seed_index'], 0)
        self.assertEqual(validation, 'primary_reliable')
        self.assertEqual(support, 1)
        self.assertEqual(cluster_count, 2)

    def test_lower_ranked_candidates_cannot_replace_primary(self):
        candidates = [{'score': 0.680}, {'score': 0.621}, {'score': 0.610}]
        reliable = [
            {'seed_index': 1, 'x': 21.4, 'y': -8.3,
             'yaw': math.radians(-97.5)},
            {'seed_index': 2, 'x': 24.0, 'y': -12.4,
             'yaw': math.radians(82.7)},
        ]

        winner, validation, _, cluster_count = choose_ndt_winner(
            reliable, candidates, 0.75, math.radians(10.0))

        self.assertIsNone(winner)
        self.assertEqual(validation, 'primary_not_reliable')
        self.assertEqual(cluster_count, 2)

    def test_even_one_fallback_cluster_cannot_replace_primary(self):
        candidates = [{'score': 0.790}, {'score': 0.757}, {'score': 0.655}]
        reliable = [
            {'seed_index': 1, 'x': 21.417, 'y': -8.316,
             'yaw': math.radians(-97.28)},
            {'seed_index': 2, 'x': 21.409, 'y': -8.316,
             'yaw': math.radians(-98.16)},
        ]

        winner, validation, _, cluster_count = choose_ndt_winner(
            reliable, candidates, 0.75, math.radians(10.0))

        self.assertIsNone(winner)
        self.assertEqual(validation, 'primary_not_reliable')
        self.assertEqual(cluster_count, 1)


if __name__ == '__main__':
    unittest.main()
