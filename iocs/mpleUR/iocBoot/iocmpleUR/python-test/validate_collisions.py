import numpy as np
from scipy.spatial.transform import Rotation
import time

from mple import MpleClient, Pose, SceneObject, SceneObjects, SceneShape

PREFIX = "mpleUR:"
PROBE = "Probe"

# Upright pose: upper arm and forearm vertical. Base rotated so box orientations are not axis aligned.
Q = np.array([0.6, -np.pi / 2, 0.0, -np.pi / 2, 0.4, 0.0])

# UR5e DH parameters, expressed in the UR controller base frame (URDF "base")
DH_D = [0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996]
DH_A = [0.0, -0.425, -0.3922, 0.0, 0.0, 0.0]
DH_ALPHA = [np.pi / 2, 0.0, 0.0, np.pi / 2, -np.pi / 2, 0.0]

SWEEP_START = 0.25
SWEEP_STEP = 0.002
TOL = 0.005

results = []


def dh_frames(q):
    T = np.eye(4)
    frames = [T]
    for th, d, a, al in zip(q, DH_D, DH_A, DH_ALPHA):
        ct, st, ca, sa = np.cos(th), np.sin(th), np.cos(al), np.sin(al)
        T = T @ np.array([
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0.0, sa, ca, d],
            [0.0, 0.0, 0.0, 1.0],
        ])
        frames.append(T)
    return frames


def check(name, ok, detail):
    print(f"[{'PASS' if ok else 'FAIL'}] {name}: {detail}")
    results.append(ok)


def place(mple, shape_type, center, dims, R=np.eye(3)):
    rz, ry, rx = Rotation.from_matrix(R).as_euler("ZYX")
    obj = SceneObject(
        name=PROBE,
        pose=Pose(tuple(float(c) for c in center), (float(rx), float(ry), float(rz))),
        shapes=[SceneShape(shape_type, Pose((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)), tuple(dims))],
    )
    revision = mple.get_scene_objects().revision
    mple.set_scene_objects(SceneObjects(objects=[obj], revision=revision))


def validate(mple):
    r = mple.validate(Q)
    return bool(r["valid"]), str(r["description"])


def forearm_onset(mple, shape_type, dims, R, center, direction):
    for s in np.arange(SWEEP_START, 0.0, -SWEEP_STEP):
        place(mple, shape_type, center + s * direction, dims, R)
        valid, desc = validate(mple)
        if not valid:
            return s, desc
    return None, "no collision found"


def run(mple):
    frames = dh_frames(Q)
    o2, o3 = frames[2][:3, 3], frames[3][:3, 3]
    tool0 = frames[6][:3, 3]

    fk = mple.compute_fk(Q)
    if not fk["valid"]:
        print(f"ComputeFK failed: {fk['description']}")
        return
    fk_pos = np.array(fk["poses"], dtype=float)[:3]
    err = np.linalg.norm(fk_pos - tool0)
    check("DH tool0 matches ComputeFK", err < 1e-3, f"err={err:.2e} m")
    if err >= 1e-3:
        return

    mid = 0.5 * (o2 + o3)
    u = (o3 - o2) / np.linalg.norm(o3 - o2)
    axis = frames[2][:3, 2]
    v = np.cross(u, axis)
    v /= np.linalg.norm(v)

    # Step 2: sphere around the forearm midpoint
    place(mple, "sphere", mid, (0.05,))
    valid, desc = validate(mple)
    check("sphere at forearm midpoint collides with forearm", not valid and "forearm" in desc, desc)
    time.sleep(0.5)

    # Step 3: sphere far away
    place(mple, "sphere", (2.0, 2.0, 2.0), (0.05,))
    valid, desc = validate(mple)
    check("sphere far away is valid", valid, desc)
    time.sleep(0.5)

    # Step 4: sphere at tool0
    place(mple, "sphere", tool0, (0.01,))
    valid, desc = validate(mple)
    check("sphere at tool0 hits EE point check", not valid and "end effector" in desc, desc)
    time.sleep(0.5)

    # Step 5a: coal box extents/orientation vs forearm, compared against a sphere of equal reach
    a, t = 0.05, 0.02
    M = np.column_stack([v, axis, np.cross(v, axis)])
    M_rot = M @ Rotation.from_euler("z", 90, degrees=True).as_matrix()

    s_sph, desc = forearm_onset(mple, "sphere", (a,), np.eye(3), mid, v)
    print(f"  sphere r={a} onset: {s_sph} ({desc})")
    if s_sph is None:
        check("forearm sphere sweep", False, desc)
        return
    print(f"  apparent forearm radius: {s_sph - a:.3f} m")

    s_box, desc = forearm_onset(mple, "box", (2 * a, t, t), M, mid, v)
    print(f"  box long axis along sweep onset: {s_box} ({desc})")
    check("box half-extent along sweep matches sphere radius",
          s_box is not None and abs(s_box - s_sph) < TOL and "forearm" in desc,
          f"box={s_box}, expected~{s_sph:.3f}")

    s_rot, desc = forearm_onset(mple, "box", (2 * a, t, t), M_rot, mid, v)
    expected = s_sph - a + t / 2
    print(f"  box rotated 90 deg onset: {s_rot} ({desc})")
    check("rotated box uses thin extent along sweep",
          s_rot is not None and abs(s_rot - expected) < TOL and "forearm" in desc,
          f"box={s_rot}, expected~{expected:.3f}")
    time.sleep(0.5)

    # Step 5b: EE point check box extents/orientation at tool0
    dims = (0.2, 0.1, 0.06)
    half = np.array(dims) / 2
    eps = 0.005
    R_generic = Rotation.from_euler("ZYX", [0.7, 0.5, 0.3]).as_matrix()
    for R, label in [(np.eye(3), "identity"), (R_generic, "rx,ry,rz=0.3,0.5,0.7")]:
        for i in range(3):
            for inside in (True, False):
                offset = half[i] + (-eps if inside else eps)
                place(mple, "box", tool0 - offset * R[:, i], dims, R)
                _, desc = validate(mple)
                hit = "end effector" in desc
                check(f"EE box {label} axis {'xyz'[i]} {'inside' if inside else 'outside'}",
                      hit == inside, desc)
    time.sleep(0.5)


def main():
    mple = MpleClient(f"{PREFIX}mple:")
    original = mple.get_scene_objects()
    try:
        run(mple)
    finally:
        original.revision = mple.get_scene_objects().revision
        mple.set_scene_objects(original)
        print("Original scene restored")

    print(f"{sum(results)}/{len(results)} checks passed")


if __name__ == "__main__":
    main()
