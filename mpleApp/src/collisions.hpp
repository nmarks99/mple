#pragma once
#include <optional>
#include <string>
#include <vector>

#include "system_model.hpp"
#include "scene.hpp"

namespace mple {

class CollisionChecker {
  public:
    CollisionChecker(const SystemModel& sys, const SceneDescription& scene);

    // Scratch space for check(). Create one per request/thread and reuse it across calls.
    struct Workspace {
        pinocchio::Data data;
        pinocchio::GeometryData geom_data;
    };
    Workspace make_workspace() const;

    // Checks the robot at joint state q against the scene objects and against itself for the
    // self-collision pairs in sys.geom_model. Returns "<robot geometry> collides with <scene object>"
    // or "<robot geometry> collides with <robot geometry> (self)" on the first collision found.
    std::optional<std::string> check(const std::vector<double>& q, Workspace& ws) const;

    const pinocchio::GeometryModel& geometry_model() const { return geom_model_; }

  private:
    const SystemModel& sys_;
    pinocchio::GeometryModel geom_model_;
    std::size_t n_robot_geoms_;
    std::vector<std::string> geom_owner_;
};

} // namespace mple
