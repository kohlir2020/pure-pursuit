from typing import List

import numpy as np

from Spline import Spline
from Util import *


class PurePursuit(object):
    robot_pose = Pose(0, 0, 0)
    lookahead_distance = 0.0
    track_width = 0.0
    path = Spline
    discrete_path = []
    last_closest_point_idx = 0
    last_lookahead_frac_inx = 0

    def __init__(self, lookahead_distance: float, path: Spline, resolution: int, track_width):
        # TODO: Support passing in a list of splines / maybe something that will append splines
        #  into one big spline at the time of discretization?
        
        self.lookahead_distance = lookahead_distance
        self.path = path
        self.track_width = track_width
        for i in range(resolution):
            coord = path.get_xy(i / resolution)
            theta = path.angle_at(i / resolution)
            new_pose = Pose(coord[0], coord[1], theta)
            self.discrete_path.append(new_pose)

    def update_pose(self, robot_pose: Pose):
        self.robot_pose = robot_pose

    def closest_point_idx(self, pose: Pose) -> int:
        min_dist = pose.distance(self.discrete_path[self.last_closest_point_idx])
        min_idx = self.last_closest_point_idx

        for i in range(self.last_closest_point_idx, len(self.discrete_path)):
            if pose.distance(self.discrete_path[i]) < min_dist:
                min_dist = pose.distance(self.discrete_path[i])
                min_idx = self.last_closest_point_idx

        self.last_closest_point_idx = min_idx
        return min_idx

    def calc_lookahead_point(self) -> Pose:
        robot_pose = self.robot_pose
        closest_idx = self.closest_point_idx(robot_pose)
        path: List[Pose] = self.discrete_path
        intersect_t = 0
        fractional_idx = 0
        point = Pose(0, 0, 0)
        for i in range(math.floor(self.last_lookahead_frac_inx), len(path) - 1):
            seg_start = [path[i].x, path[i].y]
            seg_end = [path[i + 1].x, path[i + 1].y]
            c = [robot_pose.x, robot_pose.y]
            r = self.lookahead_distance

            d = [seg_end[0] - seg_start[0], seg_end[1] - seg_start[1]]
            f = [seg_start[0] - c[0], seg_start[1] - c[1]]

            a = np.dot(d, d)
            b = 2 * np.dot(f, d)
            c = np.dot(f, f) - pow(r, 2)

            discriminant = b * b - 4 * a * c
            if discriminant < 0:
                # no intersection
                pass
            else:
                discriminant = math.sqrt(discriminant)
                t1 = (-b - discriminant) / (2 * a)
                t2 = (-b + discriminant) / (2 * a)
                if 0 <= t1 <= 1:
                    fractional_idx = i + t1
                    point = Pose(seg_start[0] + t1 * d[0], seg_start[1] + t1 * d[1],
                                 path[closest_idx].theta)
                    break
                if 0 <= t2 <= 1:
                    fractional_idx = i + t2
                    point = Pose(seg_start[0] + t2 * d[0], seg_start[1] + t2 * d[1],
                                 path[closest_idx].theta)
                    break

        self.last_lookahead_frac_inx = fractional_idx
        return point

    def left_right_speeds(self, target_vel, curvature) -> []:
        left = target_vel * (2.0 + curvature * self.track_width) / 2.0
        right = target_vel * (2.0 - curvature * self.track_width) / 2.0
        return [left, right]

    def curvature_to_lookahead(self, lookahead_point: Pose):
        # positive curvature is a right turn for these calcs
        robot_pose = self.robot_pose

        a = -math.tan(robot_pose.theta)
        b = 1
        c = math.tan(robot_pose.theta) * robot_pose.x - robot_pose.y
        # point line distance formula, x = |ax + by +c| / sqrt(a^2 + b^2), where x is an unsigned distance
        x = abs(a * lookahead_point.x + b * lookahead_point.y + c) / math.sqrt(pow(a, 2) + pow(b, 2))
        side = np.sign(math.sin(robot_pose.theta) * (lookahead_point.x - robot_pose.x)
                       - math.cos(robot_pose.theta) * (lookahead_point.y - robot_pose.y))

        curvature = 2.0 * x / (math.pow(self.lookahead_distance, 2))
        return side * curvature

