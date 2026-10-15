#include <epicsGuard.h>
#include <epicsThread.h>
#include <epicsUnitTest.h>
#include <testMain.h>
#include <cmath>

#include <trajectory.hpp>

constexpr bool equal(double a, double b, double tol=1e-9) {
    if (std::fabs(a - b) < tol) {
        return true;
    }
    return false;
}

void print_vector(const std::vector<double>& vec) {
    for (size_t i = 0; i < vec.size(); i++) {
        if (i == vec.size()-1) {
            printf("%.12lf\n", vec[i]);
        } else {
            printf("%.12lf,", vec[i]);
        }
    }
}

MAIN(testTrajectory) {
    testPlan(0);

    GeometricPath path({
        {0.0, 1.0, 0.0, 0.0, 0.0, 0.0},
        {0.0, 3.0, 0.0, 0.0, 0.0, 0.0},
        {0.0, 6.0, 0.0, 0.0, 0.0, 0.0},
        {0.0, 10.0, 0.0, 0.0, 0.0, 0.0},
        {0.0, 15.0, 0.0, 0.0, 0.0, 0.0}
    });
    testOk1(static_cast<int>(path.length()) == 14);

    auto q = path.sample(5.2);
    testOk1(equal(q[1], 6.2));

    const double vmax = 1.0;
    const double amax = 99999.0;
    const double dt = 0.002;
    auto traj = create_trajectory(path, vmax, amax, dt);
    testDiag("traj.size() = %ld\n", traj.size());

    for (const auto& q : traj) {
        print_vector(q);
    }

    return testDone();
}
