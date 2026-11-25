#include <array>
#include <cmath>
#include "mple_system_model.hpp"
#include "kinematics.hpp"

#include <epicsUnitTest.h>
#include <testMain.h>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/multibody.hpp>
#include <pinocchio/parsers/urdf.hpp>

using namespace mple;

MAIN(testFK) {
    testPlan(10);

    std::array<double, 6> test_joints = {-M_PI/2, -M_PI/2, -M_PI/2, -M_PI/2, M_PI/2, 0.0};

    constexpr const char* urdf_path = MPLE_TEST_DESCRIPTION_DIR "/ur5e.urdf";
    SystemModel system;

    pinocchio::urdf::buildModel(urdf_path, system.model);
    system.base_frame_id = system.model.getFrameId("base", pinocchio::BODY);
    system.ee_frame_id = system.model.getFrameId("tool0", pinocchio::BODY);
    system.g_rb = system.model.frames[system.base_frame_id].placement;

    const auto pose = forward_kinematics(system, std::vector<double>(test_joints.begin(), test_joints.end()));

    testOk1(pose.size() == 6);
    std::array expected_trans = {-0.1333, -0.4909, 0.4879};
    for (size_t i = 0; i < expected_trans.size(); ++i) {
        testOk1(std::abs(pose[i] - expected_trans[i]) < 0.002);
    }
    Eigen::Matrix3d expected_rotation;
    expected_rotation << -1.0, 0.0, 0.0,
                         0.0, 1.0, 0.0,
                         0.0, 0.0, -1.0;
    const Eigen::Matrix3d rotation = pinocchio::rpy::rpyToMatrix(pose[3], pose[4], pose[5]);
    testOk1(rotation.isApprox(expected_rotation, 1e-6));

    // UR3e wrist_3 is continuous, so Pinocchio has nq = 7 while the state has 6 values
    constexpr const char* ur3e_path = MPLE_TEST_DESCRIPTION_DIR "/ur3e.urdf";
    SystemModel ur3e;
    pinocchio::urdf::buildModel(ur3e_path, ur3e.model);
    ur3e.base_frame_id = ur3e.model.getFrameId("base", pinocchio::BODY);
    ur3e.ee_frame_id = ur3e.model.getFrameId("tool0", pinocchio::BODY);
    ur3e.g_rb = ur3e.model.frames[ur3e.base_frame_id].placement;
    init_joint_limits(ur3e);
    testOk(ur3e.model.nq == 7 && ur3e.state_dim == 6, "UR3e has nq 7 and 6 state values");
    testOk(std::isinf(ur3e.lower_limits[5]) && std::isinf(ur3e.upper_limits[5]),
           "UR3e wrist_3 is unbounded");
    testOk(std::abs(ur3e.upper_limits[4] - 2.0 * M_PI) < 1e-9, "UR3e wrist_2 keeps its URDF limit");

    std::vector<double> state{0.3, -1.2, 1.0, -0.5, 0.7, 0.4};
    const auto p1 = forward_kinematics(ur3e, state);
    state[5] += 4.0 * M_PI;
    const auto p2 = forward_kinematics(ur3e, state);
    state[5] += M_PI / 2.0;
    const auto p3 = forward_kinematics(ur3e, state);
    const auto rot = [](const std::vector<double>& p) { return pinocchio::rpy::rpyToMatrix(p[3], p[4], p[5]); };
    testOk(rot(p1).isApprox(rot(p2), 1e-9) && std::abs(p1[0] - p2[0]) + std::abs(p1[1] - p2[1]) +
                                                         std::abs(p1[2] - p2[2]) < 1e-9,
           "wrist_3 angle and angle + 4 pi give the same pose");
    testOk(!rot(p1).isApprox(rot(p3), 1e-3), "wrist_3 angle changes the tool orientation");

    return testDone();
}
