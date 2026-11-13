from mple import MpleClient, Pose
from epics import caget, caput
import numpy as np
from numpy import cos, sin, pi

PREFIX = "mpleUR:"

mple = MpleClient(f"{PREFIX}mple:")

def joints(rad=True):
    if rad:
        return np.radians(caget(f"{PREFIX}Receive:Joints"))
    else:
        return caget(f"{PREFIX}Receive:Joints")

def pose():
    pose = caget(f"{PREFIX}Receive:Pose")
    pose[0:3] /= 1000.0
    return np.round(pose, 4)

def move(goal, speed=0.1, accel=1.0):
    result = mple.plan(goal)
    if (result["valid"]):
        mple.execute(result.trajectory, speed, accel)

def set_ball_pos(pos):
    scene = mple.get_scene_objects()
    scene["Ball1"].pose = Pose(pos, scene["Ball1"].pose.orientation)
    mple.set_scene_objects(scene)

def get_ball_pos():
    scene = mple.get_scene_objects()
    return np.round(scene["Ball1"].pose.position, 4)

def get_ball_size():
    scene = mple.get_scene_objects()
    return float(scene["Ball1"].shapes[0].dimensions[0])

def set_ball_size(radius):
    scene = mple.get_scene_objects()
    scene["Ball1"].shapes[0].dimensions = np.array([radius,])
    mple.set_scene_objects(scene)
