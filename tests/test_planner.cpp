#include <epicsUnitTest.h>
#include <testMain.h>

#include <algorithm>
#include <cmath>
#include <limits>

#include <planner.hpp>

MAIN(testPlanner) {
    testPlan(29);

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

    auto make_params = [&](std::vector<double> start, std::vector<double> goal, StateValidityChecker checker) {
        PlanParams p;
        p.start = std::move(start);
        p.goal = std::move(goal);
        p.lower_limits = lower;
        p.upper_limits = upper;
        p.is_state_valid = std::move(checker);
        return p;
    };

    // The shortest angular route from 2.8 to -2.8 crosses +-pi, outside the limits
    PlanResult long_way = plan(make_params({2.8, 0.0}, {-2.8, 0.0}, always_valid));
    testOk1(long_way.valid);
    testOk1(in_limits(long_way));
    testOk1(long_way.trajectory_flat.size() >= 4);
    testOk1(long_way.trajectory_flat.front() == 2.8);
    testOk1(long_way.trajectory_flat[long_way.trajectory_flat.size() - 2] == -2.8);

    // Obstacle band forces a detour in joint 1, which must stay within [-1, 1]
    auto band = [](const std::vector<double>& q) { return !(std::abs(q[0]) < 0.2 && q[1] > -0.8); };
    PlanResult detour = plan(make_params({-1.0, 0.0}, {1.0, 0.0}, band));
    testOk1(detour.valid);
    testOk1(in_limits(detour));

    for (const char* name : {"RRTstar", "BITstar", "ABITstar"}) {
        auto params = make_params({-1.0, 0.0}, {1.0, 0.0}, band);
        params.algo = *planner_from_string(name);
        params.planning_time = 0.2;
        PlanResult r = plan(params);
        testOk(r.valid && in_limits(r), "%s finds a detour", name);
    }
    testOk1(!planner_from_string("Bogus"));

    testOk1(!plan(make_params({3.5, 0.0}, {0.0, 0.0}, always_valid)).valid);
    testOk1(!plan(make_params({0.0, 0.0}, {0.0, 1.5}, always_valid)).valid);
    testOk1(!plan(make_params({0.0, 0.0}, {0.0, 0.0}, nullptr)).valid);

    auto bad_dims = make_params({0.0, 0.0}, {0.0, 0.0}, always_valid);
    bad_dims.lower_limits = {-1.0};
    bad_dims.upper_limits = {1.0};
    testOk1(!plan(bad_dims).valid);

    auto bad_limits = make_params({0.0, 0.0}, {0.0, 0.0}, always_valid);
    bad_limits.lower_limits = {-INFINITY, -1.0};
    testOk1(!plan(bad_limits).valid);

    for (double r : {0.0, -0.01, 1000.0, std::numeric_limits<double>::quiet_NaN()}) {
        auto params = make_params({0.0, 0.0}, {0.5, 0.0}, always_valid);
        params.resolution = r;
        testOk(!plan(params).valid, "resolution %g rejected", r);
    }

    // A thin wall on a short direct segment must be found. The extent here is about 6.3, so a
    // resolution relative to the extent would check only x = -0.045, 0.010, 0.065 and skip the wall.
    {
        auto thin_wall = [](const std::vector<double>& q) { return !(std::abs(q[0]) < 0.008 && q[1] < 0.5); };
        const PlanResult r = plan(make_params({-0.1, 0.0}, {0.12, 0.0}, thin_wall));
        double max_y = 0.0;
        for (size_t i = 1; i < r.trajectory_flat.size(); i += 2) {
            max_y = std::max(max_y, r.trajectory_flat[i]);
        }
        testOk(r.valid && max_y > 0.45, "short segment through a thin wall is rejected (max y %g)", max_y);
    }

    for (double t : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN()}) {
        auto params = make_params({0.0, 0.0}, {0.5, 0.0}, always_valid);
        params.planning_time = t;
        testOk(!plan(params).valid, "planning_time %g rejected", t);
    }

    // Unbounded joint 0, starting several turns out (multi-turn reading)
    {
        auto p = make_params({15.0, 0.0}, {-1.0, 0.0}, always_valid);
        p.lower_limits = {-INFINITY, -1.0};
        p.upper_limits = {INFINITY, 1.0};
        const PlanResult r = plan(p);
        const auto& t = r.trajectory_flat;
        testOk(r.valid, "unbounded joint plans from a multi-turn start");
        testOk(t.size() >= 4 && t.front() == 15.0 && t[t.size() - 2] == -1.0,
               "unbounded joint path keeps the exact start and goal angles");
        bool in_range = true;
        for (size_t i = 0; i < t.size(); i += 2) {
            in_range = in_range && t[i] >= -1.0 - 2.0 * M_PI && t[i] <= 15.0 + 2.0 * M_PI;
        }
        testOk(in_range, "unbounded joint stays within one turn of start and goal");
    }

    {
        auto p = make_params({15.0, 0.0}, {-1.0, 0.0}, always_valid);
        p.lower_limits = {-INFINITY, -1.0};
        testOk(!plan(p).valid, "half-unbounded limits are rejected");
    }

    {
        auto p = make_params({0.0, 0.0}, {0.0, 0.0}, always_valid);
        p.lower_limits = {-INFINITY, -1.0};
        p.upper_limits = {INFINITY, 1.0};
        p.start[0] = std::numeric_limits<double>::quiet_NaN();
        testOk(!plan(p).valid, "NaN start is rejected");
    }

    return testDone();
}
