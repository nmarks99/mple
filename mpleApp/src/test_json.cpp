#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include "collisions.hpp"
#include "obstacles_schema.hpp"

using json = nlohmann::json;

int main(int argc, char* argv[]) {

    if (argc != 2) {
        printf("Provide JSON file\n");
        return 1;
    }

    // Create a CollisionWorld with obstacles from a json file
    auto world = CollisionWorld::from_json(argv[1]);

    std::cout << "Loaded these obstacles:\n";
    for (auto& [name, obj] : world.objects) {
        std::cout << name << std::endl;
        std::cout << obj.shape_poses.size() << std::endl;
    }
    std::cout << "\n";

    // Check these points to see if they will collide
    std::vector<Position> points = {
        Position(0,0,0),
        Position(3,3,0),
        Position(-2,2,0),
        Position(3,-2,0),
        Position(-3,-2,0),
    };
    for (auto& point : points) {
        auto collision = world.check_point(point);
        if (collision) {
            std::cout << "Hit " << *collision << std::endl;
        } else {
            std::cout << "Miss" << std::endl;
        }
    }


    return 0;
}
