#include "kinematics.hpp"

// FIX: Generalize to any system
// Not all systems will have a pose in SE3
std::vector<double> forward_kinematics(const MpleSystemModel& sys, const std::vector<double>& state) {
    const Eigen::Map<const Eigen::VectorXd> q(state.data(), state.size());
    pinocchio::Data data(sys.model);
    pinocchio::framesForwardKinematics(sys.model, data, q);
    // data.oMf is in the URDF root frame (g_re). Return the EE pose in the base frame: g_be = g_rb^-1 * g_re
    const pinocchio::SE3 pin_pose = sys.g_rb.actInv(data.oMf[sys.ee_frame_id]);

    const Eigen::Vector3d rpy = pinocchio::rpy::matrixToRpy(pin_pose.rotation());
    std::vector<double> pose = {
        pin_pose.translation().x(),
        pin_pose.translation().y(),
        pin_pose.translation().z(),
        rpy.x(),
        rpy.y(),
        rpy.z(),
    };
    return pose;
}
