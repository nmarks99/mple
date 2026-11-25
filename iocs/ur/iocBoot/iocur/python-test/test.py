from mple import MpleClient, Pose
from epics import caget, caput
import numpy as np
import sys
from numpy import cos, sin, pi

PREFIX = "ur:"

def movej(qrad, wait=False):
    q = np.degrees(qrad)
    caput(PREFIX + "Control:J1Cmd", q[0])
    caput(PREFIX + "Control:J2Cmd", q[1])
    caput(PREFIX + "Control:J3Cmd", q[2])
    caput(PREFIX + "Control:J4Cmd", q[3])
    caput(PREFIX + "Control:J5Cmd", q[4])
    caput(PREFIX + "Control:J6Cmd", q[5])
    caput(PREFIX + "Control:moveJ", 1, wait=wait)

mple = MpleClient(f"{PREFIX}mple:")

a = np.radians([-90, -90, -90, -90, 90, 0])
b = np.array([-1.36791207, -1.48039799, -1.53870574, -1.57116209,  1.56734209, 0.02813187])
c = np.array([-1.13612455, -1.48243398, -1.6891447 , -1.42271022,  1.53944874, 0.2582714 ])


PLANNER = "EITstar"
PLANNING_TIME=1.0
SPEED = 1.0
ACC = 5.0
if len(sys.argv) == 2:
    target_str = sys.argv[1]
    if target_str == "a":
        print("Planning to target a")
        result = mple.plan(a, planner=PLANNER, planning_time=PLANNING_TIME)
    elif target_str == "b":
        print("Planning to target b")
        result = mple.plan(b, planner=PLANNER, planning_time=PLANNING_TIME)
    elif target_str == "c":
        print("Planning to target c")
        result = mple.plan(c, planner=PLANNER, planning_time=PLANNING_TIME)
    else:
        print(f"Invalid target {target_str}")


    if (result.valid):
        print(result.trajectory)
        mple.execute(result.trajectory, speed=SPEED, acceleration=ACC)
    else:
        print("Failed to find a path")

