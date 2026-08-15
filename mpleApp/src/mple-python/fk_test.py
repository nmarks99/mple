from mple import Mple
import numpy as np

def main():
    mple = Mple("mpleTest:mple:")

    fk_result = mple.compute_fk([np.pi/4, 0.0])
    if (fk_result["valid"]):
        print(np.round(fk_result["poses"], 4))
    else:
        print(f"Forward kinematics failed: {fk_result["description"]}")

if __name__ == "__main__":
    main()
