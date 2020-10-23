from __future__ import annotations
import math

class Pose(object):
    x = 0.0
    y = 0.0
    theta = 0.0

    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    def __str__(self):
        return "(x, y, theta): (" + str(self.x) + ", " + str(self.y) + ", " + str(self.theta) + ")"

    @classmethod
    def copy_of(cls, waypoint: Pose):
        return cls(waypoint.x, waypoint.y, waypoint.theta)

    def distance(self, other: Pose):
        return math.hypot(self.x - other.x, self.y - other.y)


class WaypointSequence(object):
    waypoints = []
    num_waypoints = 0

    def add_waypoint(self, waypoint: Pose) -> WaypointSequence:
        self.waypoints.append(waypoint)
        self.num_waypoints = len(self.waypoints)
        return self


class TrajectoryConfig(object):
    max_v = 0.0
    max_a = 0.0
    max_j = 0.0
    start_vel = 0.0
    end_vel = 0.0
    interval = 0.0

    def __init__(self, max_v: float, max_a: float, max_j: float,
                 interval: float, start_vel: float = 0.0, end_vel: float = 0.0):
        self.max_v = max_v
        self.max_a = max_a
        self.start_vel = start_vel
        self.end_vel = end_vel
        self.interval = interval
        self.max_j = max_j
