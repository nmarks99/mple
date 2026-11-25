#pragma once
#include <cassert>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/math.hpp>
#include <pinocchio/multibody.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/geometry.hpp>

namespace mple {

// Frames:
//   r: URDF root frame. Pinocchio expresses everything in this frame (data.oMf, data.oMi,
//      geometry placements). Pinocchio's docs call it "world" or "o". It is internal to mple.
//   b: base frame, chosen by name in the adapter config (base_frame). This is mple's "world"
//      frame: all poses exposed by mple (scene objects, FK/IK, cartesian waypoints) are in it.
//      For the UR5e URDF, b ("base") is rotated pi about z relative to r.
//
// Naming: g_ab is the pose of frame b expressed in frame a, so p_a = g_ab * p_b and
// g_ab * g_bc = g_ac.
struct SystemModel {
    pinocchio::Model model;
    pinocchio::GeometryModel geom_model;
    pinocchio::FrameIndex base_frame_id;
    pinocchio::FrameIndex ee_frame_id;
    std::size_t state_dim;

    // Joint position limits in state space (one value per joint, size nv). Continuous joints
    // have -inf/+inf. Set by init_joint_limits().
    std::vector<double> lower_limits;
    std::vector<double> upper_limits;

    // Pose of the base frame in the URDF root frame. Constant because the base frame is
    // required to be rigidly attached to the root (no moving joints in between).
    // Convert root -> base with g_rb.actInv(g_rx), base -> root with g_rb * g_bx.
    pinocchio::SE3 g_rb;
};

// The mple state has one angle or length per joint (size nv). Pinocchio stores a continuous joint
// as (cos, sin), so its configuration q has size nq > nv. Only 1-DOF joints are supported.

// Sets state_dim and the state-space joint limits. Throws for joints with more than one DOF.
inline void init_joint_limits(SystemModel& sys) {
    const auto& model = sys.model;
    constexpr double inf = std::numeric_limits<double>::infinity();
    sys.state_dim = static_cast<std::size_t>(model.nv);
    sys.lower_limits.assign(sys.state_dim, -inf);
    sys.upper_limits.assign(sys.state_dim, inf);
    for (pinocchio::JointIndex i = 1; i < model.joints.size(); ++i) {
        const auto& joint = model.joints[i];
        if (joint.nv() != 1 || (joint.nq() != 1 && joint.nq() != 2)) {
            throw std::runtime_error("MPLE supports only 1-DOF joints, but joint '" + model.names[i] +
                                     "' is " + joint.shortname());
        }
        if (joint.nq() == 1) {
            sys.lower_limits[joint.idx_v()] = model.lowerPositionLimit[joint.idx_q()];
            sys.upper_limits[joint.idx_v()] = model.upperPositionLimit[joint.idx_q()];
        }
    }
}

inline Eigen::VectorXd state_to_configuration(const pinocchio::Model& model, const std::vector<double>& state) {
    assert(state.size() == static_cast<std::size_t>(model.nv));
    Eigen::VectorXd q(model.nq);
    for (pinocchio::JointIndex i = 1; i < model.joints.size(); ++i) {
        const auto& joint = model.joints[i];
        const double x = state[joint.idx_v()];
        if (joint.nq() == 2) {
            q[joint.idx_q()] = std::cos(x);
            q[joint.idx_q() + 1] = std::sin(x);
        } else {
            q[joint.idx_q()] = x;
        }
    }
    return q;
}

} // namespace mple
