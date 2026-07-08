#include <nlohmann/json.hpp>
#include <fstream>
#include <string>
#include <iostream>
#include "collisions.hpp"
#include "obstacles_schema.hpp"

using json = nlohmann::json;

CollisionObject collision_object_from_json(const std::string& name, quicktype::ObstacleValue ob) {

    std::vector<std::variant<Box, Sphere, Cylinder>> shapes;
    std::vector<Pose> shape_poses;
    for (auto& shape : ob.shapes) {
        if (shape.type == quicktype::Type::BOX) {
            shapes.push_back(Box{
                shape.dimensions[0],
                shape.dimensions[1],
                shape.dimensions[2],
            });
        } else if (shape.type == quicktype::Type::SPHERE) {
            shapes.push_back(Sphere{
                shape.dimensions[0],
            });
        } else if (shape.type == quicktype::Type::CYLINDER) {
            shapes.push_back(Cylinder {
                shape.dimensions[0],
                shape.dimensions[1],
            });
        }
        Pose shape_pose;
        if (shape.pose) {
            if (auto pos = shape.pose.value().position) {
                shape_pose.position = *pos;
            }
            if (auto ori = shape.pose.value().orientation) {
                shape_pose.orientation = *ori;
            }
        } else {
            shape_pose.position = {0, 0, 0};
            shape_pose.orientation = {0, 0, 0};
        }
        shape_poses.push_back(shape_pose);
    }

    return CollisionObject {
        name,
        Pose{ Position(ob.pose.position), Orientation(ob.pose.orientation) },
        shapes,
        shape_poses
    };
}

int main(int argc, char* argv[]) {

    if (argc != 2) {
        printf("Provide JSON file\n");
        return 1;
    }

    std::ifstream f(argv[1]);
    quicktype::Obstacles obstacles = json::parse(f);

    CollisionWorld world;

    for (auto& ob : obstacles) {
        auto coll_obj = collision_object_from_json(ob.first, ob.second);
        std::cout << coll_obj.pose.position.x << std::endl;
        world.add_object(coll_obj);
    }

    std::cout << "loaded these obstacles:\n";
    for (auto& [name, obj] : world.objects) {
        std::cout << name << std::endl;
        std::cout << obj.shape_poses.size() << std::endl;
    }

    std::vector<Position> points = {
        Position(0,0,0),
        Position(3,3,0),
        Position(-2,2,0),
        Position(3,-2,0),
        Position(-3,-2,0),
    };
    for (auto& point : points) {
        // printf("checking point: (%lf, %lf, %lf)\n", point.x, point.y, point.z);
        auto collision = world.check_point(point);
        if (collision) {
            std::cout << "Hit " << *collision << std::endl;
        } else {
            std::cout << "Miss" << std::endl;
        }
    }


    return 0;
}
