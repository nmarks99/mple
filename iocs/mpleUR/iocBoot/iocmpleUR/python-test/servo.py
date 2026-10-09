from epics import caget, caput

PREFIX = "mpleUR:"

def main():
    q = caget(f"{PREFIX}Receive:Joints")


    nelm = caget(f"{PREFIX}Control:ServoTraj.NELM")
    max_points = int(nelm / 6)
    print(max_points)

    traj = []
    for _ in range(6*1000):
        q[0] -= 0.01
        for v in q:
            traj.append(v)

    caput(f"{PREFIX}Control:ServoTraj", traj)

if __name__ == "__main__":
    main()
