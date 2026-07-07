#pragma once
#include <string>
#include <variant>
#include <vector>
#include <unordered_map>

// 3D Cartesian coordinate
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

struct CollisionObject {
    std::string name;
    Pose pose;                                               // CollisionObject pose in world frame
    std::vector<std::variant<Box, Sphere, Cylinder>> shapes; // One or more shapes
    std::vector<Pose> shape_poses;                           // Pose of each shape in CollisionObject frame
};

struct CollisionInfo {
    bool hit = false;
    std::string name;
    explicit operator bool() const {
        return hit;
    }
};

struct CollisionWorld {
    void add_object(CollisionObject obj);
    void remove_object(const std::string& name);
    CollisionInfo check_point(const Position& point) const;
    std::unordered_map<std::string, CollisionObject> objects;
};
