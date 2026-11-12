#include <cassert>
#include <memory>
#include <type_traits>
#include <variant>

#include <Eigen/Geometry>
#include <coal/shape/geometric_shapes.h>
#include <pinocchio/collision/collision.hpp>

#include "collisions.hpp"

namespace {

pinocchio::SE3 to_se3(const Pose& pose) {
    const Eigen::Matrix3d rotation = (Eigen::AngleAxisd(pose.orientation.rz, Eigen::Vector3d::UnitZ()) *
                                      Eigen::AngleAxisd(pose.orientation.ry, Eigen::Vector3d::UnitY()) *
                                      Eigen::AngleAxisd(pose.orientation.rx, Eigen::Vector3d::UnitX()))
                                         .toRotationMatrix();
    return pinocchio::SE3(rotation, Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z));
}

std::shared_ptr<coal::CollisionGeometry> to_coal(const ScenePrimitive& primitive) {
    return std::visit(
        [](const auto& p) -> std::shared_ptr<coal::CollisionGeometry> {
            using T = std::decay_t<decltype(p)>;
            if constexpr (std::is_same_v<T, Box>) {
                return std::make_shared<coal::Box>(p.width, p.height, p.depth);
            } else if constexpr (std::is_same_v<T, Sphere>) {
                return std::make_shared<coal::Sphere>(p.radius);
            } else {
                return std::make_shared<coal::Cylinder>(p.radius, p.height);
            }
        },
        primitive);
}

} // namespace

CollisionChecker::CollisionChecker(const MpleSystemModel& sys, const SceneDescription& scene)
    : sys_(sys), geom_model_(sys.geom_model), n_robot_geoms_(sys.geom_model.ngeoms) {

    // Scene poses are in the base frame (b), but Pinocchio places geometry in the URDF root
    // frame (r). Shape placement in root frame: g_rs = g_rb * g_bo * g_os
    for (const auto& [name, obj] : scene.objects) {
        const pinocchio::SE3 g_ro = sys.g_rb * to_se3(obj.pose);
        for (std::size_t i = 0; i < obj.shapes.size(); ++i) {
            const auto& shape = obj.shapes[i];
            geom_model_.addGeometryObject(pinocchio::GeometryObject(
                name + "/" + std::to_string(i), 0, g_ro * to_se3(shape.pose), to_coal(shape.primitive)));
            geom_owner_.push_back(name);
        }
    }

    for (std::size_t s = n_robot_geoms_; s < geom_model_.ngeoms; ++s) {
        for (std::size_t r = 0; r < n_robot_geoms_; ++r) {
            geom_model_.addCollisionPair(pinocchio::CollisionPair(r, s));
        }
    }
}

CollisionChecker::Workspace CollisionChecker::make_workspace() const {
    return Workspace{pinocchio::Data(sys_.model), pinocchio::GeometryData(geom_model_)};
}

std::optional<std::string> CollisionChecker::check(const std::vector<double>& q, Workspace& ws) const {
    assert(q.size() == static_cast<std::size_t>(sys_.model.nq));
    const Eigen::Map<const Eigen::VectorXd> q_vec(q.data(), q.size());
    if (!pinocchio::computeCollisions(sys_.model, ws.data, geom_model_, ws.geom_data, q_vec, true)) {
        return std::nullopt;
    }
    const auto& cp = geom_model_.collisionPairs[ws.geom_data.collisionPairIndex];
    return geom_model_.geometryObjects[cp.first].name + " collides with " +
           geom_owner_[cp.second - n_robot_geoms_];
}
