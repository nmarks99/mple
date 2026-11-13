from mple import MpleClient
from epics import caget
import numpy as np

PREFIX = "mpleUR:"
def main():
    mple = MpleClient(f"{PREFIX}mple:")

    goal = caget(f"{PREFIX}Receive:Joints")
    goal += 5
    goal = np.radians(goal)
    result = mple.plan(goal)
    print(result)
    if (result["valid"]):
        traj = np.array(result["trajectory"])
        mple.execute(traj, speed=0.1, acceleration=1.0)

if __name__ == "__main__":
    main()
