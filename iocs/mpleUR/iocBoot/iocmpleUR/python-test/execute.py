from math import radians

from epics import caget
from mple import Mple


PREFIX = "mpleUR:"


def main():
    mple = Mple(f"{PREFIX}mple:")

    q0_deg = caget(f"{PREFIX}Receive:Joints")
    q0 = [radians(q) for q in q0_deg]
    q1 = q0.copy()
    q1[0] += radians(10.0)

    print("Start (deg):", q0_deg)
    print("Target (deg):", [q0_deg[0] + 10.0, *q0_deg[1:]])
    print(mple.execute([*q0, *q1]))


if __name__ == "__main__":
    main()
