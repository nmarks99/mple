#include <cstdint>
#include <fstream>
#include <type_traits>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <nlohmann/json.hpp>
#include "planning_scene.hpp"

SceneObject SceneObject::from_quicktype(const std::string& name, const quicktype::SceneObjectValue& q_obj) {

    std::vector<SceneShape> shapes;
    for (auto& qshape : q_obj.shapes) {
        SceneShape shape;
        switch (qshape.type) {
            case quicktype::Type::BOX:
                shape.primitive = Box{
                    qshape.dimensions[0],
                    qshape.dimensions[1],
                    qshape.dimensions[2],
                };
                break;
            case quicktype::Type::SPHERE:
                shape.primitive = Sphere{
                    qshape.dimensions[0],
                };
                break;
            case quicktype::Type::CYLINDER:
                shape.primitive = Cylinder {
                    qshape.dimensions[0],
                    qshape.dimensions[1],
                };
                break;
        }

        if (qshape.pose) {
            if (auto pos = qshape.pose.value().position) {
                shape.pose.position = *pos;
            }
            if (auto ori = qshape.pose.value().orientation) {
                shape.pose.orientation = *ori;
            }
        } else {
            shape.pose.position = {0, 0, 0};
            shape.pose.orientation = {0, 0, 0};
        }
        shapes.push_back(std::move(shape));
    }

    return SceneObject {
        name,
        Pose{
            Position(q_obj.pose.position),
            Orientation(q_obj.pose.orientation)
        },
        shapes,
    };
}

// PVA structure to define objects in the planning scene.
// Each object is composed of one or more primitive shapes.
pvxs::TypeDef scene_objects_pva_type() {
    namespace M = pvxs::members;
    return pvxs::TypeDef(pvxs::TypeCode::Struct, {
        M::StructA("objects", {
            M::String("name"),
            M::Float64A("position"),
            M::Float64A("orientation"),
            M::StructA("shapes", {
                M::String("type"),
                M::Float64A("position"),
                M::Float64A("orientation"),
                M::Float64A("dimensions"),
            }),
        }),
        M::UInt64("revision")
    });
}

// TODO: finish this, and use it in onPut method to convert the
// received pvxs::Value to a SceneObject
SceneObject SceneObject::from_pvxs_value(const pvxs::Value& value) {

    SceneObject obj;

    // obj.name = value["name"].as<std::string>();

    // auto shape_type = value["type"].as<std::string>();
    // if (shape_type == "sphere") {
        // Sphere sphere;
        // sphere.radius = value["dimensions"].as<pvxs::shared_array<const double>>()[0];
        // obj.shapes.push_back(sphere);
        // auto pos = value["position"].as<pvxs::shared_array<const double>>();
        // auto ori = value["orientation"].as<pvxs::shared_array<const double>>();
        // Pose pose;
        // pose.position.x = pos[0];
        // pose.position.y = pos[1];
        // pose.position.z = pos[2];
        // pose.orientation.rx = ori[0];
        // pose.orientation.ry = ori[1];
        // pose.orientation.rz = ori[2];
        // obj.pose = pose;
    // } else {
        // // TODO: implement for other shapes
        // throw std::runtime_error("unsupported shape type" + shape_type);
    // }
//
    // Pose shape_pose{
        // Position{0.0, 0.0, 0.0},
        // Orientation{0.0, 0.0, 0.0}
    // };
    // auto shape_pos = value["shape_position"].as<pvxs::shared_array<const double>>();
    // if (!shape_pos.empty()) {
        // if (shape_pos.size() != 3) {
            // throw std::runtime_error("shape_position must contain three values");
        // }
        // shape_pose.position = {shape_pos[0], shape_pos[1], shape_pos[2]};
    // }
    // auto shape_ori = value["shape_orientation"].as<pvxs::shared_array<const double>>();
    // if (!shape_ori.empty()) {
        // if (shape_ori.size() != 3) {
            // throw std::runtime_error("shape_orientation must contain three values");
        // }
        // shape_pose.orientation = {shape_ori[0], shape_ori[1], shape_ori[2]};
    // }
    // obj.shape_poses.push_back(shape_pose);

    return obj;
}

void SceneObject::to_pvxs_value(pvxs::Value& value) const {
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

    pvxs::shared_array<pvxs::Value> shape_values(shapes.size());
    for (size_t i = 0; i < shapes.size(); ++i) {
        const auto& shape = shapes[i];
        auto& shape_value = shape_values[i];
        shape_value = value["shapes"].allocMember();
        shape_value["position"] = pvxs::shared_array<const double>{
            shape.pose.position.x,
            shape.pose.position.y,
            shape.pose.position.z
        };
        shape_value["orientation"] = pvxs::shared_array<const double>{
            shape.pose.orientation.rx,
            shape.pose.orientation.ry,
            shape.pose.orientation.rz
        };

        std::visit([&shape_value](const auto& primitive) {
            using T = std::decay_t<decltype(primitive)>;
            if constexpr (std::is_same_v<T, Sphere>) {
                shape_value["type"] = "sphere";
                shape_value["dimensions"] = pvxs::shared_array<const double>{primitive.radius};
            } else if constexpr (std::is_same_v<T, Box>) {
                shape_value["type"] = "box";
                shape_value["dimensions"] = pvxs::shared_array<const double>{
                    primitive.width,
                    primitive.height,
                    primitive.depth
                };
            } else if constexpr (std::is_same_v<T, Cylinder>) {
                shape_value["type"] = "cylinder";
                shape_value["dimensions"] = pvxs::shared_array<const double>{
                    primitive.radius,
                    primitive.height
                };
            }
        }, shape.primitive);
    }
    value["shapes"] = shape_values.freeze();
}

pvxs::Value PlanningScene::to_pvxs_value() const {
    auto top = scene_objects_pva_type().create();

    pvxs::shared_array<pvxs::Value> objects(this->objects.size());
    size_t i = 0;
    for (const auto& [name, obj] : this->objects) {
        objects[i] = top["objects"].allocMember();
        obj.to_pvxs_value(objects[i]);
        ++i;
    }
    top["objects"] = objects.freeze();
    top["revision"] = uint64_t{0};

    return top;
}

PlanningScene PlanningScene::from_json(std::istream& input) {
    using json = nlohmann::json;
    PlanningScene world{};
    quicktype::SceneObjects objects = json::parse(input);
    for (auto& ob : objects) {
        world.add_object(SceneObject::from_quicktype(ob.first, ob.second));
    }
    return world;
}

PlanningScene PlanningScene::from_json(const std::string& file_path) {
    std::ifstream file(file_path);
    if (!file) {
        throw std::invalid_argument("Invalid file path provided " + file_path);
    }
    return from_json(file);
}

void PlanningScene::add_from_json(const std::string& file_path) {
    using json = nlohmann::json;
    std::ifstream file(file_path);
    if (!file) {
        throw std::invalid_argument("Invalid file path provided " + file_path);
    }
    quicktype::SceneObjects objects = json::parse(file);
    for (auto& ob : objects) {
        add_object(SceneObject::from_quicktype(ob.first, ob.second));
    }
}

void PlanningScene::add_object(SceneObject obj) {
    std::string name = obj.name;
    objects.insert_or_assign(std::move(name), std::move(obj));
}

void PlanningScene::remove_object(const std::string& name) {
    objects.erase(name);
}

std::optional<std::string> PlanningScene::check_point(const Position& point) const {
    for (auto& [_, obj] : objects) {
        for (size_t i = 0; i < obj.shapes.size(); i++) {

            auto& pose_wo = obj.pose;
            auto& shape = obj.shapes[i];
            auto& pose_os = obj.shapes[i].pose;

            // Transformation matrix from the world to the SceneObject frame
            Eigen::Isometry3d g_wo;
            {
                auto position = Eigen::Vector3d(pose_wo.position.x,pose_wo.position.y,pose_wo.position.z);
                Eigen::Matrix3d rotation;
                rotation = Eigen::AngleAxisd(pose_wo.orientation.rz, Eigen::Vector3d::UnitZ())
                    * Eigen::AngleAxisd(pose_wo.orientation.ry, Eigen::Vector3d::UnitY())
                    * Eigen::AngleAxisd(pose_wo.orientation.rx, Eigen::Vector3d::UnitX());
                g_wo = (Eigen::Translation3d(position) * rotation).matrix();
            }

            // Transformation matrix from SceneObject frame to this shape's frame
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

            if (auto* box = std::get_if<Box>(&shape.primitive)) {
                Eigen::Vector3d half_extents(box->width / 2.0, box->height / 2.0, box->depth / 2.0);
                if ((ps.array().abs() <= half_extents.array()).all()) {
                    return obj.name;
                }
            } else if (auto* sphere = std::get_if<Sphere>(&shape.primitive)) {
                if (ps.squaredNorm() <= (sphere->radius*sphere->radius)) {
                    return obj.name;
                }
            } else if (auto* cylinder = std::get_if<Cylinder>(&shape.primitive)) {
                // FIX:
                (void)cylinder;
                printf("[check_point] cylinder check not implemented yet\n");
            }
        }
    }
    return std::nullopt;
}
