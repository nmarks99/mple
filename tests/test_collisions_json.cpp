#include <epicsUnitTest.h>
#include <testMain.h>

#include <sstream>

#include "scene.hpp"

MAIN(testJson) {
    testPlan(5);

    // hardcode a valid json file here instead of
    // trying to load one from the filesystem for this test
    std::istringstream input(R"json(
{
    "Ball1": {
        "pose": {
            "position": [-2, 2, 0],
            "orientation": [0, 0, 0]
        },
        "shapes": [{
            "type": "sphere",
            "dimensions": [1.0],
            "pose": {
                "position": [0, 0, 0],
                "orientation": [0, 0, 0]
            }
        }]
    },
    "Box1": {
        "pose": {
            "position": [1, 2, 0],
            "orientation": [0, 0, 0]
        },
        "shapes": [{
            "type": "box",
            "dimensions": [2, 2, 2],
            "pose": {
                "position": [2, 1, 0],
                "orientation": [0, 0, 0.7]
            }
        }]
    },
    "Box2": {
        "pose": {
            "position": [3, -2, 0],
            "orientation": [0, 0, 0]
        },
        "shapes": [{
            "type": "box",
            "dimensions": [2, 2, 2]
        }]
    }
}
)json");
    const auto world = SceneDescription::from_json(input);

    auto collision = world.ee_check_point({0, 0, 0});
    testOk(!collision, "Miss at (0, 0, 0)");

    collision = world.ee_check_point({3, 3, 0});
    testOk(collision && *collision == "Box1", "Hit Box1 at (3, 3, 0)");

    collision = world.ee_check_point({-2, 2, 0});
    testOk(collision && *collision == "Ball1", "Hit Ball1 at (-2, 2, 0)");

    collision = world.ee_check_point({3, -2, 0});
    testOk(collision && *collision == "Box2", "Hit Box2 at (3, -2, 0)");

    collision = world.ee_check_point({-3, -2, 0});
    testOk(!collision, "Miss at (-3, -2, 0)");

    return testDone();
}
