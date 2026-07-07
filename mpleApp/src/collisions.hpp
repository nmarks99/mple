#pragma once
#include <optional>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

// All lengths are in meters and angles in radians

// 3D Cartesian coordinates
struct Position {
    double x, y, z;
};

// Euler angles
struct Orientation {
    double rx, ry, rz;
};

struct Pose {
    Position position;
    Orientation orientation;
};

// Shape primitives
struct Box {
    double width;
    double height;
    double depth;
};
struct Sphere {
    double radius;
};
struct Cylinder {
    double radius;
    double height;
};

// A collision object made up of one or more primitive shapes
struct CollisionObject {
    std::string name;
    Pose pose;                                               // CollisionObject pose in world frame
    std::vector<std::variant<Box, Sphere, Cylinder>> shapes; // One or more shapes
    std::vector<Pose> shape_poses;                           // Pose of each shape in CollisionObject frame
};

struct CollisionWorld {
    // Adds an object to the world. If an object with the same name is already
    // present, it is overwritten.
    void add_object(CollisionObject obj);

    // Removes an object from the world
    void remove_object(const std::string& name);

    // Checks if a point collides with any of the objects in the world
    // If there was a collision, the returned optional will contain the
    // string name of the object we collided with.
    std::optional<std::string> check_point(const Position& point) const;

    // Map of the objects in the world, keyed by name
    std::unordered_map<std::string, CollisionObject> objects;
};
