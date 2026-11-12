#pragma once
#include <optional>
#include <string>
#include <vector>

#include "mple_system_model.hpp"
#include "scene.hpp"

class CollisionChecker {
  public:
    CollisionChecker(const MpleSystemModel& sys, const SceneDescription& scene);

    // Scratch space for check(). Create one per request/thread and reuse it across calls.
    struct Workspace {
        pinocchio::Data data;
        pinocchio::GeometryData geom_data;
    };
    Workspace make_workspace() const;

    // Checks the robot at joint state q against the scene objects. Returns
    // "<robot geometry> collides with <scene object>" on the first collision found.
    std::optional<std::string> check(const std::vector<double>& q, Workspace& ws) const;

    const pinocchio::GeometryModel& geometry_model() const { return geom_model_; }

  private:
    const MpleSystemModel& sys_;
    pinocchio::GeometryModel geom_model_;
    std::size_t n_robot_geoms_;
    std::vector<std::string> geom_owner_;
};
