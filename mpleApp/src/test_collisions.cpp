#include "collisions.hpp"
#include <iostream>
#include <cmath>

int main(int argc, char* argv[]) {

    if (argc != 4) {
        std::cout << "not enough inputs\n";
        return EXIT_FAILURE;
    }

    Position point;
    try {
        point = {
            std::stod(argv[1]),
            std::stod(argv[2]),
            std::stod(argv[3]),
        };
    } catch (...) {
        std::cout << "invalid input\n";
        return EXIT_FAILURE;
    }

    CollisionWorld world;

    world.add_object(CollisionObject {
        "box1",
        Pose { {1, 2, 0}, {0, 0, 0} },
        { Box {2, 2, 2} },
        { Pose { {2, 1, 0}, {0, 0, M_PI/4.0} } }
    });

    world.add_object(CollisionObject {
        "box2",
        Pose { {3, -2, 0}, {0, 0, 0} },
        { Box {2, 2, 2} },
        { Pose { {0, 0, 0}, {0, 0, 0} } }
    });

    auto collision = world.check_point(point);
    if (collision) {
        std::cout << "Hit " << *collision << std::endl;
    } else {
        std::cout << "Miss" << std::endl;
    }

    return EXIT_SUCCESS;
}
