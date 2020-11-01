import math

from Util import TrajectoryConfig

#Write a trajectory generator that takes in a 1D path (length), position,
#   max velocity, and max acceleration and builds a trapezoidal trajectory
#   path to send to pure pursuit

#class for a Segment object => Used to hold kinematoc data about a
#a specific segment of the trajectory
class Segment(object):
    pos = 0.0
    start_vel = 0.0;
    end_vel = 0.0
    acc = 0.0
    dt = 0.0
    length = 0.0

    def __init__(self, pos, vel, acc, dt):
        self.pos = pos
        self.vel = vel
        self.acc = acc
        self.dt = dt

#class for a Trajectory object
class Trajectory(object):
    segments = []; #an array to hold segment objects

    def __init__(self, length: int):
        for i in range(length):
            self.segments.append(Segment())

    def scale(self, factor: float):
        for i in range(len(self.segments)):
            self.segments[i].pos *= factor
            self.segments[i].vel *= factor
            self.segments[i].acc *= factor

    def relative_pos(self, start: float):
        for i in range(len(self.segments)):
            self.segment[i].pos += start

    def get_index(self, indx: int):
        return self.segment[indx]

#function to generate path
'''
def generate(config: TrajectoryConfig, start_vel: float, goal_pos: float,
             goal_vel: float):
'''
def generate(config: TrajectoryConfig, start_vel: float, start_heading: float,
             goal_pos: float, goal_vel: float, goal_heading: float):
    
    #uses a trapezoidal motion profile
    start_discont = 0.5 * math.pow(start_vel, 2) / config.max_a
    end_discont = 0.5 * math.pow(goal_vel, 2) / config.max_a

    new_max_vel = min(config.max_a, math.sqrt(config.max_a * goal_pos - start_discont - end_discont))

    rampup_time = (new_max_vel - start_vel) / config.max_a
    rampup_dist = start_vel * rampup_time + 0.5 * config.max_a * math.pow(rampup_time, 2)
    rampdown_time = (new_max_vel - goal_vel) / config.max_a
    rampdown_dist = new_max_vel * rampdown_time - 0.5 * config.max_a * math.pow(rampdown_time, 2)
    cruise_dist = goal_pos - rampup_time - rampdown_time

    seg_length = int((rampup_time + rampdown_time + (cruise_dist / new_max_vel)) / config.interval + 0.5)

    traj = Trajectory(seg_length)
    last = Segment()

    last.pos = 0.0
    last.vel = start_vel
    last.acc = 0
    last.dt = 0

    for i in range(seg_length):

        traj.get_index(i).vel = new_max_vel
        traj.segments[i].pos = (last.vel + traj.segments[i].vel) / 2.0 * config.interval + last.pos
        traj.segments[i].acc = (traj.segments[i].vel - last.vel) / config.interval

        last = traj.segments[i]

    return traj
