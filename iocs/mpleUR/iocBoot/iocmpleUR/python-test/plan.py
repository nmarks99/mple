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
        traj = np.array(result["trajectory"]).reshape(-1, 6)

if __name__ == "__main__":
    main()
