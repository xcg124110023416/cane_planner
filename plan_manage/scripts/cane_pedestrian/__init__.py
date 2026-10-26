"""Deterministic pedestrian motion, independent of ROS and RViz."""
import math

import numpy as np


class Pedestrian:
    END_BEHAVIORS = ("stop", "disappear", "return", "repeat")
    MAX_SPEED = 10.0
    MIN_DURATION = 0.1
    MAX_DURATION = 3600.0

    def __init__(self, pid, start, vel, size, period, mode="trigger",
                 end_behavior="disappear", start_on_goal=True, yaw=None):
        self.pid = pid
        self.start = np.array(start, dtype=float)
        self.vel_mag = np.array(vel, dtype=float)
        self.pos = self.start.copy()
        self.vel = self.vel_mag.copy()
        self.size = np.array(size, dtype=float)
        self.period = float(period)
        self.mode = mode
        self.end_behavior = end_behavior
        self.start_on_goal = start_on_goal
        self.yaw = float(yaw if yaw is not None else math.atan2(vel[1], vel[0]))
        if mode == "controlled":
            if (self.start.shape != (3,) or self.vel_mag.shape != (3,)
                    or self.size.shape != (3,)
                    or not np.all(np.isfinite([self.start, self.vel_mag, self.size]))
                    or np.any(self.size <= 0) or not math.isfinite(self.yaw)):
                raise ValueError("Position, velocity and size must be finite; size must be positive")
            if (not math.isfinite(self.period)
                    or not self.MIN_DURATION <= self.period <= self.MAX_DURATION):
                raise ValueError("One-way duration must be between 0.1 and 3600 seconds")
            if np.linalg.norm(self.vel_mag[:2]) > self.MAX_SPEED + 1e-9 or self.vel_mag[2] != 0:
                raise ValueError("Walking speed must be at most 10 m/s, with zero vertical velocity")
            if end_behavior not in self.END_BEHAVIORS:
                raise ValueError("Unknown end behavior: " + end_behavior)
        self.elapsed = 0.0
        self.active = False
        self.paused = False
        self.hidden = False
        self.phase = "waiting" if mode == "controlled" else "legacy_" + mode
        self.trail = []
        self.max_trail = 30
        if mode == "controlled":
            self.vel = np.zeros(3)

    @property
    def speed(self):
        magnitude = float(np.linalg.norm(self.vel_mag[:2]))
        # Controlled velocity was already bounded; remove roundoff at exactly 10 m/s.
        return min(self.MAX_SPEED, magnitude) if self.mode == "controlled" else magnitude

    @property
    def visible(self):
        if self.mode == "controlled":
            return not self.hidden
        return self.mode != "trigger" or self.active

    def reset(self):
        self.elapsed = 0.0
        self.pos = self.start.copy()
        self.vel = np.zeros(3)
        self.active = self.paused = self.hidden = False
        self.phase = "waiting"
        self.trail.clear()

    def trigger(self):
        """Reset to start and begin walking, including after completion."""
        self.reset()
        self.vel = self.vel_mag.copy()
        self.active = True
        self.phase = "walking"

    def resume_or_start(self):
        if self.paused:
            self.paused = False
            self.active = True
            self.phase = "walking"
            self._controlled_step(0.0)
        else:
            self.trigger()

    def pause(self):
        if self.active:
            self.active = False
            self.paused = True
            self.phase = "paused"
            self.vel = np.zeros(3)

    def _controlled_step(self, dt):
        if not self.active:
            return
        self.elapsed += dt
        t = self.elapsed
        direction = 1.0
        if self.end_behavior in ("return", "repeat"):
            if self.end_behavior == "repeat":
                t %= 2.0 * self.period
            if t >= self.period:
                distance_time = max(0.0, 2.0 * self.period - t)
                direction = -1.0
            else:
                distance_time = t
            finished = self.end_behavior == "return" and t >= 2.0 * self.period
        else:
            distance_time = min(t, self.period)
            finished = t >= self.period
        self.pos = self.start + self.vel_mag * distance_time
        self.vel = self.vel_mag * direction
        if finished:
            self.active = False
            self.vel = np.zeros(3)
            self.phase = "finished"
            self.hidden = self.end_behavior == "disappear"

    def step(self, dt):
        if self.mode == "controlled":
            self._controlled_step(dt)
        elif self.mode == "trigger":
            if not self.active:
                self.vel = np.zeros(3)
                return
            self.elapsed += dt
            if self.elapsed >= self.period:
                self.active = False
                self.vel = np.zeros(3)
                return
            self.vel = self.vel_mag.copy()
            self.pos = self.start + self.vel_mag * self.elapsed
        elif self.mode == "loop":
            self.elapsed += dt
            self.vel = self.vel_mag.copy()
            self.pos = self.start + self.vel_mag * self.elapsed
            if self.elapsed >= self.period:
                self.elapsed = 0.0
                self.pos = self.start.copy()
        else:  # Legacy patrol: start is the center, period is a full cycle.
            self.elapsed = (self.elapsed + dt) % self.period
            if self.elapsed < self.period / 4.0:
                self.vel = self.vel_mag
            elif self.elapsed < self.period * 3.0 / 4.0:
                self.vel = -self.vel_mag
            else:
                self.vel = self.vel_mag
            self.pos = self.pos + self.vel * dt
        self.trail.append(self.pos.copy())
        if len(self.trail) > self.max_trail:
            self.trail.pop(0)
