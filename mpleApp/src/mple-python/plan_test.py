from mple import Mple
import numpy as np

def main():
    mple = Mple("mpleTest:mple:")

    result = mple.plan([0.0, 0.0], [np.pi/2, np.pi/4], 10)
    print(result)
    if (result["valid"]):
        traj = np.asarray(result["trajectory"]).reshape(-1, 2)
        mple.execute(traj.flatten())

if __name__ == "__main__":
    main()
