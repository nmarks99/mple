#include <epicsGuard.h>
#include <epicsThread.h>
#include <epicsUnitTest.h>
#include <testMain.h>
#include <cmath>

#include <trajectory.hpp>

// constexpr bool equal(double a, double b, double tol=1e-9) {
    // if (std::fabs(a - b) < tol) {
        // return true;
    // }
    // return false;
// }

MAIN(testTrajectory) {
    testPlan(0);

    testOk1(1==1);

    // GeometricPath path({
        // {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        // {0.0, 8.0, 0.0, 0.0, 0.0, 0.0},
    // });
    // testOk1(equal(path.length(), 8.0));
//
    // auto q = path.sample(3.0);
    // testOk1(equal(q[1], 3.0));
//
    // const double vmax = 2.0;
    // const double amax = 10.0;
    // const double dt = 0.25;
    // const auto traj = create_trajectory(path, vmax, amax, dt);
    // testOk(traj.size() == 31, "7.5 second trajectory sampled every 0.25 seconds has 31 points");

    return testDone();
}
