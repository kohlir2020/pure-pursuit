import math

from Util import TrajectoryConfig


class Segment(object):
    pos = 0.0
    vel = 0.0
    acc = 0.0
    jerk = 0.0
    heading = 0.0
    dt = 0.0
    x = 0.0
    y = 0.0

    def __init__(self, pos=0.0, vel=0.0, acc=0.0, jerk=0.0, heading=0.0, dt=0.0, x=0.0, y=0.0):
        self.pos = pos
        self.vel = vel
        self.acc = acc
        self.jerk = jerk
        self.heading = heading
        self.dt = dt
        self.x = x
        self.y = y

    def __str__(self):
        return "pos, vel, acc, (x, y): " + str(self.pos) + ", " + str(self.vel) + ", " + str(
            self.acc) + ", " + " (" + str(self.x) + ", " + str(self.y) + ")"


class Trajectory(object):
    segments = []

    def __init__(self, length: int):
        for i in range(length):
            self.segments.append(Segment())

    def scale(self, factor: float):
        for i in range(len(self.segments)):
            self.segments[i].pos *= factor
            self.segments[i].vel *= factor
            self.segments[i].acc *= factor
            self.segments[i].jerk *= factor

    def relative_pos(self, start: float):
        for i in range(len(self.segments)):
            self.segments[i].pos += start

    def get_index(self, idx: int) -> Segment:
        return self.segments[idx]


def second_order_filter(f1_len: int, f2_len: int, dt: float, start_vel: float, max_vel: float,
                        total_impulse: float, length: int, method: str = "trapezoidal"):
    if length <= 0:
        return None

    traj = Trajectory(length)
    last = Segment()

    last.pos = 0
    last.vel = start_vel
    last.acc = 0
    last.jerk = 0
    last.dt = dt

    f1 = []
    for i in range(length):
        f1.append(0.0)
    f1[0] = (start_vel / max_vel) * f1_len
    f2 = 0.0
    for i in range(length):
        input_: float = min(total_impulse, 1.0)
        if input_ < 1:
            input_ -= 1.0
            total_impulse = 0.0
        else:
            total_impulse -= input_

        f1_last = 0.0
        if i > 0:
            f1_last = f1[i - 1]
        else:
            f1_last = f1[0]

        f1[i] = max(0.0, min(f1_len, f1_last + input_))
        f2 = 0

        for j in range(f2_len):
            if (i - j) < 0:
                break

            f2 += f1[i - j]
        f2 = f2 / f1_len

        traj.get_index(i).vel = f2 / f2_len * max_vel

        if method == "rectangular":
            traj.segments[i].pos = traj.segments[i].vel * dt + last.pos
        elif method == "trapezoidal":
            traj.segments[i].pos = (last.vel + traj.segments[i].vel) / 2.0 * dt + last.pos

        traj.segments[i].x = traj.segments[i].pos
        traj.segments[i].y = 0

        traj.segments[i].acc = (traj.segments[i].vel - last.vel) / dt
        traj.segments[i].jerk = (traj.segments[i].acc - last.acc) / dt
        traj.segments[i].dt = dt

        last = traj.segments[i]

    return traj


def generate(config: TrajectoryConfig, start_vel: float, start_heading: float,
             goal_pos: float, goal_vel: float, goal_heading: float):
    strat = choose_strategy(start_vel, goal_vel, config.max_v)

    traj = None

    if strat == "step":
        impulse = (goal_pos / config.max_v / config.interval)
        time = math.floor(impulse)
        traj = second_order_filter(1, 1, config.interval, config.max_v, config.max_v, impulse, time, "trapezoidal")
    elif strat == "trapezoid":
        start_discount = .5 * start_vel * start_vel / config.max_a
        end_discount = .5 * goal_vel * goal_vel / config.max_a

        adjusted_max_v = min(config.max_a, math.sqrt(config.max_a * goal_pos - start_discount - end_discount))

        t_rampup = (adjusted_max_v - start_vel) / config.max_a
        x_rampup = start_vel * t_rampup + .5 * config.max_a * t_rampup * t_rampup
        t_rampdown = (adjusted_max_v - goal_vel) / config.max_a
        x_rampdown = adjusted_max_v * t_rampdown - .5 * config.max_a * t_rampdown * t_rampdown
        x_cruise = goal_pos - x_rampdown - x_rampup

        time = int((t_rampup + t_rampdown + x_cruise / adjusted_max_v) / config.interval + .5)

        f1_length = int(math.ceil((adjusted_max_v / config.max_a) / config.interval))
        impulse = (goal_pos / adjusted_max_v) / config.interval - \
                  start_vel / config.max_a / config.interval + start_discount + end_discount

        traj = second_order_filter(f1_length, 1, config.interval, start_vel, adjusted_max_v, impulse, time,
                                   "trapezoidal")

    elif strat == "scurve":
        adjusted_max_v = min(config.max_v,
                             (-config.max_a * config.max_a
                              + math.sqrt(config.max_a * config.max_a * config.max_a * config.max_a
                                          + 4 * config.max_j * config.max_j * config.max_j * goal_pos))
                             / (2.0 * config.max_j))

        f1_length = int(math.ceil((adjusted_max_v / config.max_a) / config.interval))

        f2_length = int(math.ceil((config.max_a / config.max_j) / config.interval))

        impulse = (goal_pos / adjusted_max_v) / config.interval

        time = int(math.ceil(f1_length + f2_length + impulse))

        traj = second_order_filter(f1_length, f2_length, config.interval,
                                   0, adjusted_max_v, impulse, time, "trapezoidal")

    else:
        return None
    total_heading_change = goal_heading - start_heading
    for i in range(len(traj.segments)):
        traj.segments[i].heading = start_heading + total_heading_change \
                                   * (traj.segments[i].pos) / traj.segments[len(traj.segments) - 1].pos

    return traj


def choose_strategy(start_vel, goal_vel, max_vel):
    strat = "trapezoid"
    if start_vel == goal_vel and start_vel == max_vel:
        strat = "step"
    elif start_vel == goal_vel and start_vel == 0:
        strat = "scurve"

    return strat
