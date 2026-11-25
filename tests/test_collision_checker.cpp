#include <cmath>
#include <exception>
#include <string>
#include <vector>

#include "pinocchio/parsers/srdf.hpp"
#include "pinocchio/parsers/urdf.hpp"

#include <epicsUnitTest.h>
#include <testMain.h>

#include "collisions.hpp"
#include "system_model.hpp"
#include "scene.hpp"

using namespace mple;

namespace {

SceneDescription single_object(const std::string& name, const Position& pos, const ScenePrimitive& primitive) {
    SceneDescription scene;
    scene.add_object(SceneObject{name, Pose{pos, {0, 0, 0}}, {SceneShape{primitive, Pose{{0, 0, 0}, {0, 0, 0}}}}});
    return scene;
}

bool ends_with(const std::string& s, const std::string& suffix) {
    return s.size() >= suffix.size() && s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
}

std::vector<double> configuration_to_state(const pinocchio::Model& model, const Eigen::VectorXd& q) {
    std::vector<double> state(model.nv);
    for (pinocchio::JointIndex i = 1; i < model.joints.size(); ++i) {
        const auto& joint = model.joints[i];
        state[joint.idx_v()] = joint.nq() == 2 ? std::atan2(q[joint.idx_q() + 1], q[joint.idx_q()])
                                               : q[joint.idx_q()];
    }
    return state;
}

// Elbow folded so the forearm hits the shoulder
const std::vector<double> q_folded{0.0, -M_PI / 2.0, 3.0, 0.0, 0.0, 0.0};

void test_ur_self_collision(const std::string& robot) {
    const std::string urdf_path = MPLE_TEST_DESCRIPTION_DIR "/" + robot + ".urdf";
    const std::string srdf_path = MPLE_TEST_DESCRIPTION_DIR "/ur.srdf";
    SystemModel sys;
    try {
        pinocchio::urdf::buildModel(urdf_path, sys.model);
        pinocchio::urdf::buildGeom(sys.model, urdf_path, pinocchio::COLLISION, sys.geom_model,
                                   MPLE_TEST_DESCRIPTION_DIR);
        sys.geom_model.addAllCollisionPairs();
        pinocchio::srdf::removeCollisionPairs(sys.model, sys.geom_model, srdf_path);
        pinocchio::srdf::loadReferenceConfigurations(sys.model, srdf_path);
    } catch (const std::exception& e) {
        testAbort("Failed to load %s: %s", robot.c_str(), e.what());
    }
    try {
        init_joint_limits(sys);
    } catch (const std::exception& e) {
        testAbort("%s: %s", robot.c_str(), e.what());
    }
    testOk(sys.state_dim == 6, "%s has 6 state values (nq %d)", robot.c_str(), sys.model.nq);
    testOk(sys.geom_model.collisionPairs.size() == 14, "%s: SRDF leaves 14 of 21 self-collision pairs",
           robot.c_str());

    const CollisionChecker c(sys, SceneDescription{});
    auto ws = c.make_workspace();
    for (const char* name : {"home", "up", "test_configuration"}) {
        const auto it = sys.model.referenceConfigurations.find(name);
        if (it == sys.model.referenceConfigurations.end()) {
            testFail("%s: group state '%s' missing from SRDF", robot.c_str(), name);
            continue;
        }
        const auto hit = c.check(configuration_to_state(sys.model, it->second), ws);
        if (hit) {
            testDiag("%s", hit->c_str());
        }
        testOk(!hit, "%s: group state '%s' has no self-collision", robot.c_str(), name);
    }
    testOk(sys.model.referenceConfigurations.size() == 3, "%s: SRDF has 3 group states", robot.c_str());

    const auto hit = c.check(q_folded, ws);
    testOk(hit.has_value(), "%s: folded elbow self-collides", robot.c_str());
    if (hit) {
        testDiag("%s", hit->c_str());
    }
    testOk(hit && ends_with(*hit, " (self)"), "%s: result marks self-collision", robot.c_str());
}

} // namespace

MAIN(testCollisionChecker) {
    testPlan(29);

    const std::string urdf_path = MPLE_TEST_DESCRIPTION_DIR "/ur5e.urdf";
    SystemModel sys;
    try {
        pinocchio::urdf::buildModel(urdf_path, sys.model);
        pinocchio::urdf::buildGeom(sys.model, urdf_path, pinocchio::COLLISION, sys.geom_model,
                                   MPLE_TEST_DESCRIPTION_DIR);
    } catch (const std::exception& e) {
        testAbort("Failed to load robot model: %s", e.what());
    }
    sys.base_frame_id = sys.model.getFrameId("base", pinocchio::BODY);
    sys.ee_frame_id = sys.model.getFrameId("tool0", pinocchio::BODY);
    sys.state_dim = static_cast<std::size_t>(sys.model.nv);
    sys.g_rb = sys.model.frames[sys.base_frame_id].placement;

    const Eigen::Matrix3d rz_pi = Eigen::Vector3d(-1.0, -1.0, 1.0).asDiagonal();
    testOk(sys.g_rb.rotation().isApprox(rz_pi, 1e-9) && sys.g_rb.translation().isZero(1e-9),
           "UR5e base frame is rotated pi about z relative to the URDF root");

    // At q = 0 the UR5e forearm lies along -x of the base frame at z ~= 0.16, spanning
    // x = -0.425 to -0.817. Positions below are in the base frame.
    SceneDescription scene;
    scene.add_object(SceneObject{"front", Pose{{-0.6, 0.0, 0.16}, {0, 0, 0}},
                                 {SceneShape{Sphere{0.1}, Pose{{0, 0, 0}, {0, 0, 0}}}}});
    scene.add_object(SceneObject{"behind", Pose{{0.6, 0.0, 0.16}, {0, 0, 0}},
                                 {SceneShape{Cylinder{0.1, 0.2}, Pose{{0, 0, 0}, {0, 0, 0}}}}});
    // Object rotated pi/2 about z with the shape offset (-0.3, 0, 0) in the object frame puts
    // the shape at (-0.6, 0, 0.16) in the base frame. Ignoring the object rotation would put it
    // at (-0.9, 0.3, 0.16), clear of the arm.
    scene.add_object(SceneObject{"composed", Pose{{-0.6, 0.3, 0.16}, {0, 0, M_PI / 2.0}},
                                 {SceneShape{Box{0.1, 0.1, 0.1}, Pose{{-0.3, 0, 0}, {0, 0, 0}}}}});

    const CollisionChecker checker(sys, scene);
    const std::size_t n_robot = sys.geom_model.ngeoms;
    testOk(checker.geometry_model().ngeoms == n_robot + 3, "one geometry per scene shape");
    testOk(checker.geometry_model().collisionPairs.size() == n_robot * 3,
           "collision pairs are robot vs scene only");

    const std::vector<double> q_zero(sys.model.nq, 0.0);
    std::vector<double> q_pan = q_zero;
    q_pan[0] = M_PI;

    {
        const CollisionChecker c(sys, single_object("front", {-0.6, 0.0, 0.16}, Sphere{0.1}));
        auto ws = c.make_workspace();
        const auto hit = c.check(q_zero, ws);
        testOk(hit.has_value(), "sphere on the forearm collides");
        if (hit) {
            testDiag("%s", hit->c_str());
        }
        testOk(hit && ends_with(*hit, " collides with front"), "result names the scene object");
        testOk(!c.check(q_pan, ws), "sphere is clear after rotating shoulder pan by pi");
    }

    {
        const CollisionChecker c(sys, single_object("behind", {0.6, 0.0, 0.16}, Cylinder{0.1, 0.2}));
        auto ws = c.make_workspace();
        testOk(!c.check(q_zero, ws), "cylinder behind the base does not collide");
        testOk(c.check(q_pan, ws).has_value(), "cylinder collides after rotating shoulder pan by pi");
    }

    {
        SceneDescription composed;
        composed.add_object(scene.objects.at("composed"));
        const CollisionChecker c(sys, composed);
        auto ws = c.make_workspace();
        testOk(c.check(q_zero, ws).has_value(), "object and shape poses compose correctly");
    }

    {
        const CollisionChecker c(sys, SceneDescription{});
        auto ws = c.make_workspace();
        testOk(!c.check(q_zero, ws), "empty scene never collides");
    }

    // Ignoring g_rb places the scene in the URDF root frame, which mirrors it in x and y
    {
        SystemModel sys_root = sys;
        sys_root.g_rb.setIdentity();
        const CollisionChecker c(sys_root, single_object("front", {-0.6, 0.0, 0.16}, Sphere{0.1}));
        auto ws = c.make_workspace();
        testOk(!c.check(q_zero, ws), "without g_rb the front sphere misses");
    }

    test_ur_self_collision("ur5e");
    test_ur_self_collision("ur3e");

    SystemModel sys_self = sys;
    try {
        sys_self.geom_model.addAllCollisionPairs();
        pinocchio::srdf::removeCollisionPairs(sys_self.model, sys_self.geom_model,
                                              MPLE_TEST_DESCRIPTION_DIR "/ur.srdf");
    } catch (const std::exception& e) {
        testAbort("Failed to load SRDF: %s", e.what());
    }

    {
        const CollisionChecker c(sys_self, single_object("front", {-0.6, 0.0, 0.16}, Sphere{0.1}));
        testOk(c.geometry_model().collisionPairs.size() == 14 + n_robot,
               "checker keeps self pairs and adds scene pairs");
    }

    {
        const CollisionChecker c(sys, SceneDescription{});
        auto ws = c.make_workspace();
        testOk(!c.check(q_folded, ws), "without SRDF self-collision is not checked");
    }

    return testDone();
}
