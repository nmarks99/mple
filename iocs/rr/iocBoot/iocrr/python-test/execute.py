from math import radians
import numpy as np
from epics import caget
import time
from mple import MpleClient

PREFIX = "rr:"

# Arm pointing to the upper left. The direct path in either direction
# sweeps through Box1 or Box2, so the planner must fold the elbow.
GOAL_DEG = [-340.0, 90.0]


def main():
    mple = MpleClient(f"{PREFIX}mple:")

    goal = np.radians(GOAL_DEG)
    plan = mple.plan(goal)
    print(f"Plan: valid={plan['valid']}, {plan['description']}")
    if not plan["valid"]:
        return

    waypoints = np.array(plan["trajectory"])
    print(f"{waypoints.size // 2} waypoints (deg):")
    print(np.degrees(waypoints.reshape(-1, 2)))

    validate = mple.validate(waypoints)
    print(f"Validate: valid={validate['valid']}, {validate['description']}")
    if not validate["valid"]:
        return

    speed = 1.0
    accel = speed*10
    result = mple.execute(waypoints, speed, accel)
    print(f"Execute: error_code={result['error_code']}, {result['description']}")
    if result["error_code"] != 0:
        return

    time.sleep(0.5)
    while caget(f"{PREFIX}Moving") == 1:
        time.sleep(0.1)
    print(f"J1={caget(f'{PREFIX}J1')}, J2={caget(f'{PREFIX}J2')}")

if __name__ == "__main__":
    main()
