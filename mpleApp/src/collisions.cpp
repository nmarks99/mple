#include "collisions.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>

void CollisionWorld::add_object(CollisionObject obj) {
    // std::string name = std::move(obj.name);
    std::string name = obj.name;
    objects.insert_or_assign(std::move(name), std::move(obj));
}

void CollisionWorld::remove_object(const std::string& name) {
    objects.erase(name);
}

CollisionInfo CollisionWorld::check_point(const Position& point) const {
    for (auto& [_, obj] : objects) {
        for (size_t i = 0; i < obj.shapes.size(); i++) {

            auto& pose_wo = obj.pose;
            auto& shape = obj.shapes[i];
            auto& pose_os = obj.shape_poses[i];

            Eigen::Isometry3d g_wo;
            {
                auto position = Eigen::Vector3d(pose_wo.position.x,pose_wo.position.y,pose_wo.position.z);
                Eigen::Matrix3d rotation;
                rotation = Eigen::AngleAxisd(pose_wo.orientation.rz, Eigen::Vector3d::UnitZ())
                    * Eigen::AngleAxisd(pose_wo.orientation.ry, Eigen::Vector3d::UnitY())
                    * Eigen::AngleAxisd(pose_wo.orientation.rx, Eigen::Vector3d::UnitX());
                g_wo = (Eigen::Translation3d(position) * rotation).matrix();
            }
            Eigen::Isometry3d g_os;
            {
                auto position = Eigen::Vector3d(pose_os.position.x,pose_os.position.y,pose_os.position.z);
                Eigen::Matrix3d rotation;
                rotation = Eigen::AngleAxisd(pose_os.orientation.rz, Eigen::Vector3d::UnitZ())
                    * Eigen::AngleAxisd(pose_os.orientation.ry, Eigen::Vector3d::UnitY())
                    * Eigen::AngleAxisd(pose_os.orientation.rx, Eigen::Vector3d::UnitX());
                g_os = (Eigen::Translation3d(position) * rotation).matrix();
            }
            Eigen::Isometry3d g_ws = g_wo * g_os;

            if (auto* box = std::get_if<Box>(&shape)) {
                Eigen::Vector3d pw(point.x, point.y, point.z);
                // printf("Point in world frame: (%lf,%lf,%lf)\n", pw.x(), pw.y(), pw.z());
                Eigen::Vector3d ps = g_ws.inverse() * pw;
                // printf("Point in shape frame: (%lf,%lf,%lf)\n", ps.x(), ps.y(), ps.z());
                Eigen::Vector3d half_extents(box->width / 2.0, box->height / 2.0, box->depth / 2.0);

                if ((ps.array().abs() <= half_extents.array()).all()) {
                    return CollisionInfo{true, obj.name};
                }
            }
        }
    }
    return CollisionInfo{false, ""};
}
