#include <cstddef>
#include <exception>
#include <memory>
#include <string>

#include <coal/shape/geometric_shapes.h>

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/collision/collision.hpp"
#include "pinocchio/parsers/urdf.hpp"

#include <epicsUnitTest.h>
#include <testMain.h>

MAIN(testPinocchioCollisions) {
    testPlan(3);

    const std::string urdf_filename = MPLE_TEST_DESCRIPTION_DIR "/ur5e.urdf";

    pinocchio::Model model;
    pinocchio::GeometryModel geom_model;
    try {
        pinocchio::urdf::buildModel(urdf_filename, model);
        pinocchio::urdf::buildGeom(model, urdf_filename, pinocchio::COLLISION, geom_model,
                                   MPLE_TEST_DESCRIPTION_DIR);
    } catch (const std::exception& e) {
        testAbort("Failed to load robot model: %s", e.what());
    }

    pinocchio::Data data(model);

    testOk1(geom_model.ngeoms == 7);

    const std::size_t n_robot_geoms = geom_model.ngeoms;

    auto sphere = std::make_shared<coal::Sphere>(0.1);
    pinocchio::SE3 sphere_placement(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.0, 0.0, 0.05));
    const pinocchio::GeomIndex sphere_id =
        geom_model.addGeometryObject(pinocchio::GeometryObject("sphere", 0, sphere_placement, sphere));

    for (std::size_t i = 0; i < n_robot_geoms; ++i) {
        geom_model.addCollisionPair(pinocchio::CollisionPair(i, sphere_id));
    }

    pinocchio::GeometryData geom_data(geom_model);
    const Eigen::VectorXd q = pinocchio::neutral(model);

    bool hit = pinocchio::computeCollisions(model, data, geom_model, geom_data, q, false);
    testOk(hit, "sphere at base collides");
    for (std::size_t k = 0; k < geom_model.collisionPairs.size(); ++k) {
        if (geom_data.collisionResults[k].isCollision()) {
            const auto& cp = geom_model.collisionPairs[k];
            testDiag("%s collides with %s", geom_model.geometryObjects[cp.first].name.c_str(),
                     geom_model.geometryObjects[cp.second].name.c_str());
        }
    }

    geom_model.geometryObjects[sphere_id].placement.translation() << 1.0, 1.0, 1.0;
    hit = pinocchio::computeCollisions(model, data, geom_model, geom_data, q, false);
    testOk(!hit, "sphere far away does not collide");

    return testDone();
}
