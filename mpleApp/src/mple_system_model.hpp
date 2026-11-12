#pragma once
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/math.hpp>
#include <pinocchio/multibody.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/geometry.hpp>

// Frames:
//   r: URDF root frame. Pinocchio expresses everything in this frame (data.oMf, data.oMi,
//      geometry placements). Pinocchio's docs call it "world" or "o". It is internal to mple.
//   b: base frame, chosen by name in the adapter config (base_frame). This is mple's "world"
//      frame: all poses exposed by mple (scene objects, FK/IK, cartesian waypoints) are in it.
//      For the UR5e URDF, b ("base") is rotated pi about z relative to r.
//
// Naming: g_ab is the pose of frame b expressed in frame a, so p_a = g_ab * p_b and
// g_ab * g_bc = g_ac.
struct MpleSystemModel {
    pinocchio::Model model;
    pinocchio::GeometryModel geom_model;
    pinocchio::FrameIndex base_frame_id;
    pinocchio::FrameIndex ee_frame_id;
    std::size_t state_dim;

    // Pose of the base frame in the URDF root frame. Constant because the base frame is
    // required to be rigidly attached to the root (no moving joints in between).
    // Convert root -> base with g_rb.actInv(g_rx), base -> root with g_rb * g_bx.
    pinocchio::SE3 g_rb;
};

