from math import radians
import numpy as np
from epics import caget
import time
from mple import Mple

PREFIX = "mpleUR:"

def main():
    mple = Mple(f"{PREFIX}mple:")

    q0_deg = caget(f"{PREFIX}Receive:Joints")
    q0 = np.array([radians(q) for q in q0_deg])
    q1 = q0.copy()
    q1 += radians(10.0)

    mple.execute([*q0, *q1])

    time.sleep(0.1)
    while (caget(f"{PREFIX}Control:Moving")):
        time.sleep(0.1)

    q0_deg = caget(f"{PREFIX}Receive:Joints")
    q0 = np.array([radians(q) for q in q0_deg])
    q1 = q0.copy()
    q1 -= radians(10.0)

    mple.execute([*q0, *q1])

if __name__ == "__main__":
    main()
