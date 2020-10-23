import matplotlib.pyplot as plt
import numpy as np

import PathGeneration
from PurePursuitController import PurePursuit
from Util import *


class DriveBase(object):
    pose = Pose(0, 0, 0)
    drive_base = 0.0
    max_v = 0.0

    def __init__(self, start_pose: Pose, drive_base, max_v):
        self.pose = start_pose
        self.drive_base = drive_base
        self.max_v = max_v

    def simulate_step(self, vel_left, vel_right, dt):
        if vel_left > self.max_v:
            vel_left = self.max_v
        if vel_right > self.max_v:
            vel_right = self.max_v

        r = self.drive_base / 2 * (vel_right + vel_left) / (vel_right - vel_left)
        omega = (vel_right - vel_left) / self.drive_base

        x_icc = self.pose.x - r * np.sin(self.pose.theta)
        y_icc = self.pose.y + r * np.cos(self.pose.theta)
        # theta = theta + d_theta
        theta = self.pose.theta + omega * dt
        x = np.cos(omega * dt) * (self.pose.x - x_icc) - np.sin(omega * dt) * (self.pose.y - y_icc) + x_icc
        y = np.sin(omega * dt) * (self.pose.x - x_icc) + np.cos(omega * dt) * (self.pose.y - y_icc) + y_icc

        new_pose = Pose(x, y, theta)
        self.pose = new_pose


if __name__ == "__main__":

    max_v = 8
    max_a = 2
    max_j = 16
    dt = 1 / 60.0
    drive_base_width = .3
    lookahead = .2

    path = WaypointSequence()
    path.add_waypoint(Pose(0.0, 0.0, 0.0))
    # path.add_waypoint(Pose(2.0, 3.0, math.pi/4))
    path.add_waypoint(Pose(3.0, 3.0, math.pi/4))

    config = TrajectoryConfig(max_v, max_a, max_j, dt)

    splines = PathGeneration.generate_from_path(path, config, False)

    robot = DriveBase(Pose(0, 0, 0), drive_base_width, max_v)
    pp = PurePursuit(lookahead, splines[0], 1000, drive_base_width)
    lookahead = pp.calc_lookahead_point()

    print(lookahead)
    cx = []
    cy = []
    for spline in splines:
        for i in range(100):
            coords = spline.get_xy(i / 100)
            print(coords)
            cx.append(coords[0])
            cy.append(coords[1])

    time = 0
    robot_x_positions = []
    robot_y_positions = []
    for i in range(int(dt * 6000)):
        pp.update_pose(robot.pose)
        left_right_vel = pp.left_right_speeds(max_v, pp.curvature_to_lookahead(pp.calc_lookahead_point()))
        left = np.sign(left_right_vel[0]) * min(abs(left_right_vel[0]), max_v)
        right = np.sign(left_right_vel[1]) * min(abs(left_right_vel[1]), max_v)
        # print(robot.pose)
        # print([left, right])
        robot.simulate_step(left, right, dt)
        robot_x_positions.append(robot.pose.x)
        robot_y_positions.append(robot.pose.y)

        plt.cla()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        plt.plot(cx, cy, "-r", label="course")
        plt.plot(robot_x_positions, robot_y_positions, "-b", label="trajectory")
        # plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
        plt.axis("equal")
        plt.grid(True)
        # plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
        plt.pause(.01)
