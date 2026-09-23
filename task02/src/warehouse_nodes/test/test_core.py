from __future__ import annotations

import math
import unittest

from warehouse_nodes.core import (
    FORWARD,
    STOP,
    CyclicReadings,
    RangeSample,
    detect_obstacle,
    limit_speed,
    must_stop,
    plan_motion,
)


class CoreContractTests(unittest.TestCase):
    def test_cyclic_readings_are_deterministic(self) -> None:
        readings = CyclicReadings([1.5, 0.35])
        self.assertEqual([readings.next().sequence for _ in range(4)], [0, 1, 2, 3])

        replay = CyclicReadings([1.5, 0.35])
        self.assertEqual(
            [replay.next().distance_m for _ in range(5)],
            [1.5, 0.35, 1.5, 0.35, 1.5],
        )

    def test_cyclic_readings_reject_invalid_programs(self) -> None:
        for readings in ([], [0.0], [-1.0], [math.nan], [math.inf]):
            with self.subTest(readings=readings), self.assertRaises(ValueError):
                CyclicReadings(readings)

    def test_threshold_boundary_is_blocked(self) -> None:
        sample = RangeSample(sequence=7, distance_m=0.60)
        decision = detect_obstacle(sample, stop_distance_m=0.60)

        self.assertEqual(decision.sequence, 7)
        self.assertTrue(decision.blocked)
        self.assertAlmostEqual(decision.distance_m, 0.60)

    def test_planner_stops_when_blocked_and_moves_when_clear(self) -> None:
        blocked = detect_obstacle(RangeSample(sequence=1, distance_m=0.30), 0.60)
        clear = detect_obstacle(RangeSample(sequence=2, distance_m=1.30), 0.60)

        stop = plan_motion(blocked, cruise_speed_mps=0.35)
        forward = plan_motion(clear, cruise_speed_mps=0.35)

        self.assertEqual((stop.mode, stop.speed_mps), (STOP, 0.0))
        self.assertEqual((forward.mode, forward.speed_mps), (FORWARD, 0.35))

    def test_speed_limit_and_independent_safety_rule(self) -> None:
        self.assertAlmostEqual(limit_speed(0.8, 0.5), 0.5)
        self.assertAlmostEqual(limit_speed(0.2, 0.5), 0.2)
        self.assertTrue(must_stop(0.5, 0.5))
        self.assertFalse(must_stop(0.5001, 0.5))


if __name__ == "__main__":
    unittest.main()
