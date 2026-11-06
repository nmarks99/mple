#include <cstdint>
#include <fstream>
#include <type_traits>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <nlohmann/json.hpp>
#include "collisions.hpp"

CollisionObject CollisionObject::from_quicktype(const std::string& name, quicktype::ObstacleValue ob) {

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

pvxs::TypeDef obstacles_pva_type() {
    namespace M = pvxs::members;
    return pvxs::TypeDef(pvxs::TypeCode::Struct, {
        M::StructA("obstacles", {
            M::String("name"),
            M::String("type"),
            M::Float64A("position"),
            M::Float64A("orientation"),
            M::Float64A("shape_position"),
            M::Float64A("shape_orientation"),
            M::Float64A("dimensions"),
        }),
        M::UInt64("revision")
    });
}

// TODO: finish this, and use it in onPut method to convert the
// received pvxs::Value to a CollisionObject
CollisionObject CollisionObject::from_pvxs_value(const pvxs::Value& value) {

    CollisionObject obj;

    obj.name = value["name"].as<std::string>();

    auto shape_type = value["type"].as<std::string>();
    if (shape_type == "sphere") {
        Sphere sphere;
        sphere.radius = value["dimensions"].as<pvxs::shared_array<const double>>()[0];
        obj.shapes.push_back(sphere);
        auto pos = value["position"].as<pvxs::shared_array<const double>>();
        auto ori = value["orientation"].as<pvxs::shared_array<const double>>();
        Pose pose;
        pose.position.x = pos[0];
        pose.position.y = pos[1];
        pose.position.z = pos[2];
        pose.orientation.rx = ori[0];
        pose.orientation.ry = ori[1];
        pose.orientation.rz = ori[2];
        obj.pose = pose;
    } else {
        // TODO: implement for other shapes
        throw std::runtime_error("unsupported shape type" + shape_type);
    }

    Pose shape_pose{
        Position{0.0, 0.0, 0.0},
        Orientation{0.0, 0.0, 0.0}
    };
    auto shape_pos = value["shape_position"].as<pvxs::shared_array<const double>>();
    if (!shape_pos.empty()) {
        if (shape_pos.size() != 3) {
            throw std::runtime_error("shape_position must contain three values");
        }
        shape_pose.position = {shape_pos[0], shape_pos[1], shape_pos[2]};
    }
    auto shape_ori = value["shape_orientation"].as<pvxs::shared_array<const double>>();
    if (!shape_ori.empty()) {
        if (shape_ori.size() != 3) {
            throw std::runtime_error("shape_orientation must contain three values");
        }
        shape_pose.orientation = {shape_ori[0], shape_ori[1], shape_ori[2]};
    }
    obj.shape_poses.push_back(shape_pose);

    return obj;
}

void CollisionObject::to_pvxs_value(pvxs::Value& value) const {
    if (shapes.size() != 1 || shape_poses.size() != 1) {
        throw std::runtime_error("PVA obstacles require exactly one shape and one shape pose per object");
    }

    value["name"] = name;
    value["position"] = pvxs::shared_array<const double>{
        pose.position.x,
        pose.position.y,
        pose.position.z
    };
    value["orientation"] = pvxs::shared_array<const double>{
        pose.orientation.rx,
        pose.orientation.ry,
        pose.orientation.rz
    };

    for (const auto& shape_pose : shape_poses) {
        value["shape_position"] = pvxs::shared_array<const double>{
            shape_pose.position.x,
            shape_pose.position.y,
            shape_pose.position.z
        };
        value["shape_orientation"] = pvxs::shared_array<const double>{
            shape_pose.orientation.rx,
            shape_pose.orientation.ry,
            shape_pose.orientation.rz
        };
    }

    for (const auto& shape : shapes) {
        std::visit([&value](const auto& arg) {
            using T = std::decay_t<decltype(arg)>;
            if constexpr (std::is_same_v<T, Sphere>) {
                value["type"] = "sphere";
                value["dimensions"] = pvxs::shared_array<const double>{arg.radius};
            } else if constexpr (std::is_same_v<T, Box>) {
                value["type"] = "box";
                value["dimensions"] = pvxs::shared_array<const double>{
                    arg.width,
                    arg.height,
                    arg.depth
                };
            } else if constexpr (std::is_same_v<T, Cylinder>) {
                value["type"] = "cylinder";
                value["dimensions"] = pvxs::shared_array<const double>{
                    arg.radius,
                    arg.height
                };
            }
        }, shape);
    }
}

pvxs::Value CollisionWorld::to_pvxs_value() const {
    auto top = obstacles_pva_type().create();

    pvxs::shared_array<pvxs::Value> obstacles(this->objects.size());
    size_t i = 0;
    for (const auto& [name, obj] : this->objects) {
        obstacles[i] = top["obstacles"].allocMember();
        obj.to_pvxs_value(obstacles[i]);
        ++i;
    }
    top["obstacles"] = obstacles.freeze();
    top["revision"] = uint64_t{0};

    return top;
}

CollisionWorld CollisionWorld::from_json(std::istream& input) {
    using json = nlohmann::json;
    CollisionWorld world{};
    quicktype::Obstacles obstacles = json::parse(input);
    for (auto& ob : obstacles) {
        auto coll_obj = CollisionObject::from_quicktype(ob.first, ob.second);
        world.add_object(coll_obj);
    }
    return world;
}

CollisionWorld CollisionWorld::from_json(const std::string& file_path) {
    std::ifstream file(file_path);
    if (!file) {
        throw std::invalid_argument("Invalid file path provided " + file_path);
    }
    return from_json(file);
}

void CollisionWorld::add_from_json(const std::string& file_path) {
    using json = nlohmann::json;
    std::ifstream file(file_path);
    if (!file) {
        throw std::invalid_argument("Invalid file path provided " + file_path);
    }
    quicktype::Obstacles obstacles = json::parse(file);
    for (auto& ob : obstacles) {
        auto coll_obj = CollisionObject::from_quicktype(ob.first, ob.second);
        add_object(coll_obj);
    }
}

void CollisionWorld::add_object(CollisionObject obj) {
    std::string name = obj.name;
    objects.insert_or_assign(std::move(name), std::move(obj));
}

void CollisionWorld::remove_object(const std::string& name) {
    objects.erase(name);
}

std::optional<std::string> CollisionWorld::check_point(const Position& point) const {
    for (auto& [_, obj] : objects) {
        for (size_t i = 0; i < obj.shapes.size(); i++) {

            auto& pose_wo = obj.pose;
            auto& shape = obj.shapes[i];
            auto& pose_os = obj.shape_poses[i];

            // Transformation matrix from the world to the CollisionObject frame
            Eigen::Isometry3d g_wo;
            {
                auto position = Eigen::Vector3d(pose_wo.position.x,pose_wo.position.y,pose_wo.position.z);
                Eigen::Matrix3d rotation;
                rotation = Eigen::AngleAxisd(pose_wo.orientation.rz, Eigen::Vector3d::UnitZ())
                    * Eigen::AngleAxisd(pose_wo.orientation.ry, Eigen::Vector3d::UnitY())
                    * Eigen::AngleAxisd(pose_wo.orientation.rx, Eigen::Vector3d::UnitX());
                g_wo = (Eigen::Translation3d(position) * rotation).matrix();
            }

            // Transformation matrix from CollisionObject frame to this shape's frame
            Eigen::Isometry3d g_os;
            {
                auto position = Eigen::Vector3d(pose_os.position.x,pose_os.position.y,pose_os.position.z);
                Eigen::Matrix3d rotation;
                rotation = Eigen::AngleAxisd(pose_os.orientation.rz, Eigen::Vector3d::UnitZ())
                    * Eigen::AngleAxisd(pose_os.orientation.ry, Eigen::Vector3d::UnitY())
                    * Eigen::AngleAxisd(pose_os.orientation.rx, Eigen::Vector3d::UnitX());
                g_os = (Eigen::Translation3d(position) * rotation).matrix();
            }

            // Transformation matrix from the world to this shape's frame
            Eigen::Isometry3d g_ws = g_wo * g_os;
            Eigen::Vector3d pw(point.x, point.y, point.z);
            Eigen::Vector3d ps = g_ws.inverse() * pw;
            // printf("[%s]Point in shape frame: (%lf,%lf,%lf)\n", obj.name.c_str(), ps.x(), ps.y(), ps.z());
            // printf("[%s]Point in world frame: (%lf,%lf,%lf)\n", obj.name.c_str(), pw.x(), pw.y(), pw.z());

            if (auto* box = std::get_if<Box>(&shape)) {
                Eigen::Vector3d half_extents(box->width / 2.0, box->height / 2.0, box->depth / 2.0);
                if ((ps.array().abs() <= half_extents.array()).all()) {
                    return obj.name;
                }
            } else if (auto* sphere = std::get_if<Sphere>(&shape)) {
                if (ps.squaredNorm() <= (sphere->radius*sphere->radius)) {
                    return obj.name;
                }
            } else if (auto* cylinder = std::get_if<Cylinder>(&shape)) {
                // TODO:
                printf("[check_point] cylinder check not implemented yet\n");
            }
        }
    }
    return std::nullopt;
}
