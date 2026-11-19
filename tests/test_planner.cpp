#include <epicsUnitTest.h>
#include <testMain.h>

#include <cmath>

#include <planner.hpp>

MAIN(testPlanner) {
    testPlan(0);

    const std::vector<double> lower = {-3.0, -1.0};
    const std::vector<double> upper = {3.0, 1.0};
    auto always_valid = [](const std::vector<double>&) { return true; };

    auto in_limits = [&](const PlanResult& r) {
        for (size_t i = 0; i < r.trajectory_flat.size(); ++i) {
            const size_t j = i % lower.size();
            if (r.trajectory_flat[i] < lower[j] || r.trajectory_flat[i] > upper[j]) {
                return false;
            }
        }
        return true;
    };

    // The shortest angular route from 2.8 to -2.8 crosses +-pi, outside the limits
    PlanResult long_way = plan({2.8, 0.0}, {-2.8, 0.0}, lower, upper, always_valid);
    testOk1(long_way.valid);
    testOk1(in_limits(long_way));
    testOk1(long_way.trajectory_flat.size() >= 4);
    testOk1(long_way.trajectory_flat.front() == 2.8);
    testOk1(long_way.trajectory_flat[long_way.trajectory_flat.size() - 2] == -2.8);

    // Obstacle band forces a detour in joint 1, which must stay within [-1, 1]
    auto band = [](const std::vector<double>& q) { return !(std::abs(q[0]) < 0.2 && q[1] > -0.8); };
    PlanResult detour = plan({-1.0, 0.0}, {1.0, 0.0}, lower, upper, band);
    testOk1(detour.valid);
    testOk1(in_limits(detour));

    testOk1(!plan({3.5, 0.0}, {0.0, 0.0}, lower, upper, always_valid).valid);
    testOk1(!plan({0.0, 0.0}, {0.0, 1.5}, lower, upper, always_valid).valid);
    testOk1(!plan({0.0, 0.0}, {0.0, 0.0}, {-1.0}, {1.0}, always_valid).valid);
    testOk1(!plan({0.0, 0.0}, {0.0, 0.0}, {-INFINITY, -1.0}, upper, always_valid).valid);

    return testDone();
}
