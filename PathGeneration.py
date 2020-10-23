from Util import *
from Spline import *
from typing import List
from TrajectoryGeneration import *


def generate_from_path(path: WaypointSequence, config: TrajectoryConfig, generate_trajectory: bool):
    if path.num_waypoints < 2:
        return None

    splines: List[Spline] = []
    for i in range(path.num_waypoints - 1):
        splines.append(Spline())
    spline_lengths = []
    for i in range(len(splines)):
        spline_lengths.append(0.0)
    total_distance = 0.0

    for i in range(path.num_waypoints - 1):
        splines[i] = Spline()
        start = path.waypoints[i]
        end = path.waypoints[i + 1]
        print(splines[i].arc_length)
        splines[i] = Spline.reticulate_spline(start.x, start.y, start.theta, end.x, end.y, end.theta)
        if splines[i] is None:
            return None

        spline_lengths[i] = splines[i].calculate_length()
        total_distance += spline_lengths[i]

    if generate_trajectory:
        traj = generate(config, 0.0, path.waypoints[0].theta, total_distance, 0.0, path.waypoints[0].theta)

        cur_spline = 0
        cur_spline_start_pos = 0.0
        length_of_splines_finished = 0.0

        for i in range(len(traj.segments)):
            cur_pos = traj.segments[i].pos
            found_spline = False

            while not found_spline:
                cur_pos_relative = cur_pos - cur_spline_start_pos
                if cur_pos_relative <= spline_lengths[cur_spline]:
                    percentage = splines[cur_spline].get_percentage_for_distance(cur_pos_relative)

                    traj.segments[i].heading = splines[cur_spline].angle_at(percentage)
                    coords = splines[cur_spline].get_xy(percentage)

                    traj.segments[i].x = coords[0]
                    traj.segments[i].y = coords[1]
                    found_spline = True
                elif cur_spline < len(splines) - 1:
                    length_of_splines_finished += spline_lengths[cur_spline]
                    cur_spline_start_pos = length_of_splines_finished
                    cur_spline += 1
                else:
                    traj.segments[i].heading = splines[len(splines) - 1].angle_at(1.0)
                    coords = splines[len(splines) - 1].get_xy(1.0)
                    traj.segments[i].x = coords[0]
                    traj.segments[i].y = coords[1]
                    found_spline = True
        return traj
    else:
        return splines
