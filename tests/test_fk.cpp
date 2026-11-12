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

MAIN(testFK) {
    testPlan(0);

    std::array<double, 6> test_joints = {-M_PI/2, -M_PI/2, -M_PI/2, -M_PI/2, M_PI/2, 0.0};

    constexpr const char* urdf_path = MPLE_TEST_DESCRIPTION_DIR "/ur5e.urdf";
    MpleSystemModel system;

    pinocchio::urdf::buildModel(urdf_path, system.model);
    system.base_frame_id = system.model.getFrameId("base", pinocchio::BODY);
    system.ee_frame_id = system.model.getFrameId("tool0", pinocchio::BODY);

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

    return testDone();
}
