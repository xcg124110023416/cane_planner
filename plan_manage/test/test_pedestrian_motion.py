#!/usr/bin/env python3
"""Check timing and position contracts without a ROS master."""
import math
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from cane_pedestrian import Pedestrian
from cane_pedestrian.scene_io import default_state_path, dumps_scene, loads_scene, write_scene


class PedestrianMotionTest(unittest.TestCase):
    def pedestrian(self, behavior="return", vel=(0.0, 0.4, 0.0)):
        return Pedestrian(0, [2, -2, 0], vel, [0.5, 0.5, 1.7], 5.0,
                          mode="controlled", end_behavior=behavior)

    def test_waiting_is_a_stationary_visible_obstacle(self):
        p = self.pedestrian()
        p.step(20.0)
        np.testing.assert_allclose(p.pos, p.start)
        np.testing.assert_allclose(p.vel, 0)
        self.assertTrue(p.visible)
        self.assertEqual(p.phase, "waiting")

    def test_return_takes_two_one_way_durations(self):
        p = self.pedestrian()
        p.trigger()
        p.step(5.0)
        np.testing.assert_allclose(p.pos, [2, 0, 0])
        np.testing.assert_allclose(p.vel, [0, -0.4, 0])
        p.step(2.5)
        np.testing.assert_allclose(p.pos, [2, -1, 0])
        p.step(2.5)
        np.testing.assert_allclose(p.pos, p.start)
        np.testing.assert_allclose(p.vel, 0)
        self.assertEqual(p.phase, "finished")
        self.assertTrue(p.visible)

    def test_pause_during_return_resumes_without_restarting(self):
        p = self.pedestrian()
        p.trigger()
        p.step(7.0)
        pos = p.pos.copy()
        p.pause()
        p.step(100.0)
        np.testing.assert_allclose(p.pos, pos)
        np.testing.assert_allclose(p.vel, 0)
        self.assertTrue(p.visible)
        p.resume_or_start()
        np.testing.assert_allclose(p.vel, [0, -0.4, 0])
        p.step(1.0)
        np.testing.assert_allclose(p.pos, [2, -1.2, 0])

    def test_terminal_tick_clamps_to_endpoint(self):
        for behavior, endpoint in (("stop", [2, 0, 0]),
                                   ("disappear", [2, 0, 0]),
                                   ("return", [2, -2, 0])):
            with self.subTest(behavior=behavior):
                p = self.pedestrian(behavior)
                p.trigger()
                p.step(123.4)
                np.testing.assert_allclose(p.pos, endpoint)
                np.testing.assert_allclose(p.vel, 0)
                self.assertEqual(p.visible, behavior != "disappear")

    def test_repeat_preserves_overshoot_and_reverses(self):
        p = self.pedestrian("repeat")
        p.trigger()
        p.step(26.0)  # Two cycles plus six seconds: one second into return.
        np.testing.assert_allclose(p.pos, [2, -0.4, 0])
        np.testing.assert_allclose(p.vel, [0, -0.4, 0])
        self.assertTrue(p.active)
        p.step(4.0)
        np.testing.assert_allclose(p.pos, p.start)
        np.testing.assert_allclose(p.vel, [0, 0.4, 0])

    def test_reset_and_restart_restore_disappeared_person(self):
        p = self.pedestrian("disappear")
        p.trigger()
        p.step(6)
        p.reset()
        self.assertTrue(p.visible)
        self.assertEqual(p.phase, "waiting")
        np.testing.assert_allclose(p.pos, p.start)
        p.resume_or_start()
        p.step(1)
        np.testing.assert_allclose(p.pos, [2, -1.6, 0])

    def test_stationary_person_retains_selected_heading(self):
        p = Pedestrian(0, [0, 0, 0], [0, 0, 0], [0.5, 0.5, 1.7], 2,
                       mode="controlled", end_behavior="return", yaw=math.pi / 2)
        p.trigger()
        p.step(4)
        np.testing.assert_allclose(p.pos, p.start)
        self.assertEqual(p.yaw, math.pi / 2)
        self.assertTrue(p.visible)

    def test_legacy_trigger_and_teleport_loop_keep_their_semantics(self):
        p = Pedestrian(0, [0, 0, 0], [1, 0, 0], [0.5, 0.5, 1.7], 2)
        self.assertFalse(p.visible)
        p.trigger()
        p.step(1)
        np.testing.assert_allclose(p.pos, [1, 0, 0])
        p.step(1)
        self.assertFalse(p.visible)
        p = Pedestrian(0, [0, 0, 0], [1, 0, 0], [0.5, 0.5, 1.7], 2, mode="loop")
        p.step(2)
        np.testing.assert_allclose(p.pos, p.start)
        self.assertTrue(p.visible)

    def test_invalid_controlled_geometry_and_timing_are_rejected(self):
        for period in (0, -1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                Pedestrian(0, [0, 0, 0], [1, 0, 0], [1, 1, 1], period, mode="controlled")
        with self.assertRaises(ValueError):
            self.pedestrian("teleport_typo")
        with self.assertRaises(ValueError):
            self.pedestrian(vel=(float("nan"), 0, 0))


class PedestrianSceneTest(unittest.TestCase):
    def test_maximum_speed_round_trips_at_different_headings(self):
        for yaw in np.linspace(-math.pi, math.pi, 17):
            p = Pedestrian(0, [0, 0, 0], [10 * math.cos(yaw), 10 * math.sin(yaw), 0],
                           [1, 1, 1], 1, mode="controlled", yaw=yaw)
            loaded, _ = loads_scene(dumps_scene(p, "world"), "world")
            self.assertLessEqual(loaded.speed, 10)
            np.testing.assert_allclose(loaded.vel_mag, p.vel_mag)

    def test_round_trip_restores_start_settings_and_clear_state(self):
        p = Pedestrian(0, [4, -3, 0], [0.4, 0, 0], [0.5, 0.5, 1.7], 3,
                       mode="controlled", end_behavior="repeat", start_on_goal=False)
        p.trigger()
        p.step(2)
        for placed in (True, False):
            loaded, present = loads_scene(dumps_scene(p, "world", placed), "world")
            self.assertEqual(present, placed)
            np.testing.assert_allclose(loaded.pos, p.start)
            np.testing.assert_allclose(loaded.vel, 0)
            self.assertEqual(loaded.phase, "waiting")
            self.assertEqual(loaded.end_behavior, "repeat")
            self.assertFalse(loaded.start_on_goal)

    def test_previous_exports_without_frame_metadata_are_supported(self):
        old = """custom_pedestrian:
  - {mode: controlled, start: [2, 1, 0], yaw: 1.57, speed: 0.6,
     duration: 2, size: [0.5, 0.5, 1.7], end_behavior: return, start_on_goal: false}
"""
        loaded, placed = loads_scene(old, "world")
        self.assertTrue(placed)
        self.assertAlmostEqual(loaded.yaw, 1.57)
        self.assertEqual(loaded.end_behavior, "return")

    def test_bad_files_and_foreign_frames_are_rejected(self):
        for text in ("", "[broken", "{}", "a: []", "a: [{}]", "a: [{}]\nb: [{}]",
                     "a: [{}, {}]", "!!python/object:object {}"):
            with self.subTest(text=text), self.assertRaises(ValueError):
                loads_scene(text, "world")
        p = Pedestrian(0, [0, 0, 0], [0, 0, 0], [1, 1, 1], 1, mode="controlled")
        with self.assertRaises(ValueError):
            loads_scene(dumps_scene(p, "map"), "world")

    def test_cache_paths_separate_scenarios_frames_and_config_files(self):
        args = ("/tmp/ros", "crossing", "/tmp/a.yaml", "world")
        base = default_state_path(*args)
        self.assertEqual(base, default_state_path(*args))
        for alternative in (("/tmp/ros", "approach", "/tmp/a.yaml", "world"),
                            ("/tmp/ros", "crossing", "/tmp/a.yaml", "map"),
                            ("/tmp/ros", "crossing", "/tmp/b.yaml", "world")):
            self.assertNotEqual(base, default_state_path(*alternative))

    def test_remembered_file_is_replaced_without_leaving_temporary_files(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "settings" / "last.yaml"
            write_scene(path, "first")
            write_scene(path, "second")
            self.assertEqual(path.read_text(), "second")
            self.assertEqual(list(path.parent.iterdir()), [path])


if __name__ == "__main__":
    unittest.main()
