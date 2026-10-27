from math import radians
import numpy as np
from epics import caget
import time
from mple import Mple

PREFIX = "mpleUR:"

def build_traj(deg):
    q0_deg = caget(f"{PREFIX}Receive:Joints")
    q0 = np.array([radians(q) for q in q0_deg])
    q1 = q0.copy()
    q1 += radians(deg)
    return np.array([*q0, *q1])

def wait_motion():
    time.sleep(0.1)
    while (caget(f"{PREFIX}Control:Moving")):
        time.sleep(0.1)

def main():
    mple = Mple(f"{PREFIX}mple:")

    vel = 10.0
    acc = 100.0

    mple.execute(build_traj(10), vel, acc)
    wait_motion()
    mple.execute(build_traj(-10), vel, acc)
    wait_motion()

if __name__ == "__main__":
    main()
