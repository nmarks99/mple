#pragma once
#include <cstdint>
#include <istream>
#include <optional>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>
#include <cassert>
#include "scene_objects_schema.hpp"

#include <pvxs/nt.h>
#include <pvxs/server.h>
#include <pvxs/sharedpv.h>

// All lengths are in meters and angles in radians

// 3D Cartesian coordinates
struct Position {
    Position() = default;
    Position(const std::vector<double>& pos) {
        assert(pos.size() == 3);
        x = pos[0];
        y = pos[1];
        z = pos[2];
    }
    Position(double xi, double yi, double zi) : x(xi), y(yi), z(zi) {}
    double x, y, z;
};

// Euler angles
struct Orientation {
    Orientation() = default;
    Orientation(const std::vector<double>& ori) {
        assert(ori.size() == 3);
        rx = ori[0];
        ry = ori[1];
        rz = ori[2];
    }
    Orientation(double rxi, double ryi, double rzi) : rx(rxi), ry(ryi), rz(rzi) {}
    double rx, ry, rz;
};

struct Pose {
    Pose() = default;
    Pose(Position pos, Orientation ori) : position(pos), orientation(ori) {}
    Position position{};
    Orientation orientation{};
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

pvxs::TypeDef scene_objects_pva_type();

// A scene object made up of one or more primitive shapes
struct SceneObject {
    static SceneObject from_quicktype(const std::string& name, quicktype::SceneObjectValue ob);
    static SceneObject from_pvxs_value(const pvxs::Value& value);

    std::string name;
    Pose pose;                                               // SceneObject pose in world frame
    std::vector<std::variant<Box, Sphere, Cylinder>> shapes; // One or more shapes
    std::vector<Pose> shape_poses;                           // Pose of each shape in SceneObject frame

    void to_pvxs_value(pvxs::Value& value) const;
};

struct PlanningScene {
    static PlanningScene from_json(std::istream& input);
    static PlanningScene from_json(const std::string& file_path);

    pvxs::Value to_pvxs_value() const;

    // Adds an object to the world. If an object with the same name is already
    // present, it is overwritten.
    void add_object(SceneObject obj);

    // Adds object to the world specified in a json file
    void add_from_json(const std::string& file_path);

    // Removes an object from the world
    void remove_object(const std::string& name);

    // Checks if a point collides with any of the objects in the world
    // If there was a collision, the returned optional will contain the
    // string name of the object we collided with.
    std::optional<std::string> check_point(const Position& point) const;

    // Map of the objects in the world, keyed by name
    std::unordered_map<std::string, SceneObject> objects;
};
