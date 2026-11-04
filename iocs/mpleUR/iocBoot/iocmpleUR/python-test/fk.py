from mple import Mple
from epics import caget
import numpy as np
from scipy.spatial.transform import Rotation

PREFIX = "mpleUR:"
def main():
    mple = Mple(PREFIX+"mple:")

    joints = np.radians(caget(PREFIX + "Receive:Joints"))

    fk_result = mple.compute_fk(joints)
    if (fk_result["valid"]):
        mple_pose = np.array(fk_result["poses"], dtype=float, copy=True)
        rpy = mple_pose[3:6]
        mple_pose[3:6] = Rotation.from_euler("xyz", rpy).as_rotvec()
        print(f"From MPLE: {np.round(mple_pose, 4)}")

        pose = caget(PREFIX + "Receive:Pose")
        pose[0:3] /= 1000.0
        print(f"From  IOC: {np.round(pose, 4)}")
    else:
        print(f"Forward kinematics failed: {fk_result['description']}")

if __name__ == "__main__":
    main()
