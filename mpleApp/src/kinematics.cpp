#include "kinematics.hpp"

// FIX: Generalize to any system
// Not all systems will have a pose in SE3
std::vector<double> forward_kinematics(const MpleSystemModel& sys, const std::vector<double>& state) {
    const Eigen::Map<const Eigen::VectorXd> q(state.data(), state.size());
    pinocchio::Data data(sys.model);
    pinocchio::framesForwardKinematics(sys.model, data, q);
    const pinocchio::SE3 pin_pose = data.oMf[sys.base_frame_id].inverse() * data.oMf[sys.ee_frame_id];

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
