from mple import Mple

def main():
    mple = Mple("mpleTest:mple:")

    poses = [
        0.550, 0.000, 0.0, 0.0, 0.0, 0.0,
        0.200, -0.200, 0.0, 0.0, 0.0, 0.0,
        0.200, 0.200, 0.0, 0.0, 0.0, 0.0,
        -0.200, 0.200, 0.0, 0.0, 0.0, 0.0,
        -0.200, -0.200, 0.0, 0.0, 0.0, 0.0,
        0.550, 0.000, 0.0, 0.0, 0.0, 0.0,
    ]

    ik_result = mple.compute_ik(poses)
    if (ik_result["valid"]):
        validation_result = mple.validate(ik_result["joint_angles"])
        if (validation_result["valid"]):
            mple.execute(ik_result["joint_angles"])
        else:
            print("Invalid path")

if __name__ == "__main__":
    main()
