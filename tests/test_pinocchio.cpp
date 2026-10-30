#include <array>
#include <cmath>
#include <exception>

#include <epicsUnitTest.h>
#include <testMain.h>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/multibody.hpp>
#include <pinocchio/parsers/urdf.hpp>

MAIN(testPinocchio) {
    testPlan(0);

    std::array<double, 6> test_joints = {-M_PI/2, -M_PI/2, -M_PI/2, -M_PI/2, M_PI/2, 0.0};

    constexpr const char* urdf_path = MPLE_TEST_URDF_PATH;
    constexpr std::array<const char*, 6> joint_names = {
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint",     "wrist_2_joint",       "wrist_3_joint",
    };

    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_path, model);
    pinocchio::Data data(model);
    Eigen::VectorXd q = pinocchio::neutral(model);

    for (std::size_t i = 0; i < joint_names.size(); ++i) {
        const double angle = test_joints[i];
        const pinocchio::JointIndex jointId = model.getJointId(joint_names[i]);
        const auto& joint = model.joints[jointId];
        if (joint.nq() == 1) {
            q[joint.idx_q()] = angle;
        } else if (joint.nq() == 2) {
            q[joint.idx_q()] = std::cos(angle);
            q[joint.idx_q() + 1] = std::sin(angle);
        }
    }

    pinocchio::framesForwardKinematics(model, data, q);

    const pinocchio::FrameIndex base_frame = model.getFrameId("base", pinocchio::BODY);
    const pinocchio::FrameIndex tool_frame = model.getFrameId("tool0", pinocchio::BODY);
    const pinocchio::SE3 pose = data.oMf[base_frame].inverse() * data.oMf[tool_frame];

    const Eigen::Vector3d expected_trans(-0.1333, -0.4909, 0.4879);
    const double pos_error = (pose.translation() - expected_trans).norm();
    testOk1(pos_error < 0.002); // check position error < 2mm

    Eigen::Matrix3d expected_rot;
    expected_rot << -1.0, 0.0, 0.0,
                     0.0, 1.0, 0.0,
                     0.0, 0.0, -1.0;
    testOk1(pose.rotation().isApprox(expected_rot, 1e-6));


    return testDone();
}
