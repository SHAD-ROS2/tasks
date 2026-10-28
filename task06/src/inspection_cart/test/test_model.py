"""Contract tests: also run with the Python standard-library unittest runner."""
import math
import os
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
from inspection_cart.model import frame_position  # noqa: E402
from inspection_cart.synthetic_states import JOINT_NAMES, positions_at_tick  # noqa: E402

MODEL = Path(os.environ.get("L06_MODEL", PACKAGE / "urdf" / "cart.urdf"))


class TestTurretContract(unittest.TestCase):
    def test_tip_follows_horizontal_arc(self):
        # These are the task's independently specified expected positions (metres).
        expected = [(-math.pi/2, (0.07, -0.30, 0.34)),
                    (0.0, (0.37, 0.0, 0.34)),
                    (math.pi/2, (0.07, 0.30, 0.34))]
        for angle, target in expected:
            with self.subTest(angle=angle):
                actual = frame_position(MODEL, "probe_tip", {"turret_joint": angle})
                for observed, required in zip(actual, target):
                    self.assertAlmostEqual(observed, required, delta=1e-6)

    def test_named_moving_joints_and_literal_revolute(self):
        document = ET.parse(MODEL).getroot()
        moving = {joint.get("name") for joint in document.findall("joint")
                  if joint.get("type") != "fixed"}
        self.assertEqual(moving, set(JOINT_NAMES))
        turret = document.find("joint[@name='turret_joint']")
        self.assertEqual(turret.get("type"), "revolute")
        limit = turret.find("limit")
        self.assertAlmostEqual(float(limit.get("lower")), -math.pi/2)
        self.assertAlmostEqual(float(limit.get("upper")), math.pi/2)
        for name in ("lower", "upper", "effort", "velocity"):
            self.assertTrue(math.isfinite(float(limit.get(name))))

    def test_mobile_visual_tree_is_connected_and_has_no_world_anchor(self):
        document = ET.parse(MODEL).getroot()
        links = {link.get("name") for link in document.findall("link")}
        children = [joint.find("child").get("link") for joint in document.findall("joint")]
        self.assertEqual(len(children), len(set(children)))
        self.assertEqual(links - set(children), {"base_link"})
        self.assertNotIn("world", links)
        for frame in links:
            frame_position(MODEL, frame)
        self.assertFalse(document.findall("ros2_control"))

    def test_synthetic_source_is_repeatable_and_inside_limits(self):
        self.assertEqual(positions_at_tick(0), [0.0, 0.0, 0.0])
        self.assertAlmostEqual(positions_at_tick(50)[2], 1.2)
        self.assertAlmostEqual(positions_at_tick(150)[2], -1.2)
        for tick in range(400):
            self.assertLess(abs(positions_at_tick(tick)[2]), math.pi/2)
            self.assertAlmostEqual(positions_at_tick(tick)[2], positions_at_tick(tick + 200)[2])


if __name__ == "__main__":
    unittest.main()
