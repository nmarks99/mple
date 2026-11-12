from mple import Mple
from epics import caget, caput
import numpy as np

PREFIX = "mpleUR:"

mple = Mple(f"{PREFIX}mple:")

def get_joints(rad=True):
    if rad:
        return np.radians(caget(f"{PREFIX}Receive:Joints"))
    else:
        return caget(f"{PREFIX}Receive:Joints")

def get_pose():
    pose = caget(f"{PREFIX}Receive:Pose")
    pose[0:3] /= 1000.0
    return pose

def move(goal, speed=0.1, accel=1.0):
    result = mple.plan(goal)
    if (result["valid"]):
        mple.execute(result.trajectory, speed, accel)

print('''
from mple import Mple
from epics import caget, caput
import numpy as np

PREFIX = "mpleUR:"

mple = Mple(f"{PREFIX}mple:")

def get_joints(rad=True):
    if rad:
        return np.radians(caget(f"{PREFIX}Receive:Joints"))
    else:
        return caget(f"{PREFIX}Receive:Joints")

def get_pose():
    pose = caget(f"{PREFIX}Receive:Pose")
    pose[0:3] /= 1000.0
    return pose

def move(goal, speed=0.1, accel=1.0):
    result = mple.plan(goal)
    if (result["valid"]):
        mple.execute(result.trajectory, speed, accel)
''')
