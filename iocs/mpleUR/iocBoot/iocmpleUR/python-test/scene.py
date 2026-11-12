import time

from mple import Mple, Pose

PREFIX = "mpleUR:"
OBJECT = "Ball1"

X_MIN = -0.5 # m
X_MAX = 0.5  # m
STEP = 0.01  # m
RATE = 50.0  # Hz


def find_object_in_scene(scene, name):
    for obj in scene.objects:
        if obj.name == name:
            return obj
    raise KeyError(f"{name} not found in scene")


def main():
    mple = Mple(f"{PREFIX}mple:")

    scene = mple.get_scene_objects()
    _, y0, z0 = find_object_in_scene(scene, OBJECT).pose.position
    orientation = find_object_in_scene(scene, OBJECT).pose.orientation

    x = X_MIN
    step = STEP
    try:
        while True:
            scene = mple.get_scene_objects()
            find_object_in_scene(scene, OBJECT).pose = Pose((x, y0, z0), orientation)
            mple.set_scene_objects(scene)

            x += step
            if x >= X_MAX or x <= X_MIN:
                step = -step

            time.sleep(1.0 / RATE)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
