#include <epicsGuard.h>
#include <epicsThread.h>
#include <epicsUnitTest.h>
#include <testMain.h>
#include <iostream>
#include <cmath>

#include "mple/trajectory.hpp"

using namespace mple;

MAIN(testTrajectory) {
    testPlan(0);

    std::vector<std::vector<double>> waypoints = {
        {0, 0, 0, 0, 0, 0},
        {1, 1, 1, 1, 1, 1},
    };

    Trajectory traj = build_trajectory(waypoints, 1.0, 10.0);

    // For now, build_trajectory will use TOPP-RA's default of
    // 100 path segments for its grid-point generator, therefore
    // produces 101 points (including both end points)
    constexpr size_t DEFAULT_TOPPRA_SEGMENTS = 100;
    testOk1(traj.size() == DEFAULT_TOPPRA_SEGMENTS + 1);

    // Check that the last point reaches our target position
    for (const auto& p : traj.back().position) {
        testOk1(std::abs(p - 1.0) < 1e-9);
    }

    // Collinear, unevenly spaced waypoints. With chord-length s the spline is the straight line, so
    // the duration is that of one trapezoid on joint 0: D / v + v / a = 1.1.
    std::vector<std::vector<double>> uneven = {
        {0.0, 0.0},
        {0.1, 0.05},
        {1.0, 0.5},
    };
    Trajectory uneven_traj = build_trajectory(uneven, 1.0, 10.0);
    testOk(std::abs(uneven_traj.back().time - 1.1) < 0.05, "uneven collinear duration %g ~ 1.1",
           uneven_traj.back().time);
    bool on_line = true;
    for (const auto& pt : uneven_traj) {
        on_line = on_line && std::abs(pt.position[1] - 0.5 * pt.position[0]) < 1e-9;
    }
    testOk(on_line, "uneven collinear path stays on the line");

    // Repeated waypoints are dropped
    std::vector<std::vector<double>> repeated = {
        {0.0, 0.0},
        {0.0, 0.0},
        {1.0, 1.0},
        {1.0, 1.0},
    };
    Trajectory repeated_traj = build_trajectory(repeated, 1.0, 10.0);
    testOk1(std::abs(repeated_traj.back().position[0] - 1.0) < 1e-9 &&
            std::abs(repeated_traj.back().position[1] - 1.0) < 1e-9);

    bool threw = false;
    try {
        build_trajectory({{0.5, 0.5}, {0.5, 0.5}}, 1.0, 10.0);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    testOk(threw, "identical waypoints are rejected");

    // Step trajectory: one point per segment
    // Segment 1: trapezoid (D=2 >= v^2/a=0.1), joint 1 moves half as far
    // Segment 2: triangle (D=0.05 < 0.1)
    // Segment 3: zero length
    std::vector<std::vector<double>> step_waypoints = {
        {0.0, 0.0},
        {2.0, -1.0},
        {2.05, -1.0},
        {2.05, -1.0},
    };
    const double v = 1.0;
    const double a = 10.0;
    Trajectory steps = build_step_trajectory(step_waypoints, v, a);
    testOk1(steps.size() == 3);

    auto near = [](double x, double y) { return std::abs(x - y) < 1e-12; };

    // Each joint's trapezoid (t_acc = v_i / a_i) must cover its distance in the segment time
    auto covers = [&](const TrajectoryPoint& pt, const std::vector<double>& from) {
        for (size_t i = 0; i < pt.position.size(); ++i) {
            const double d = std::abs(pt.position[i] - from[i]);
            if (d == 0.0) {
                if (pt.velocity[i] != 0.0 || pt.acceleration[i] != 0.0) {
                    return false;
                }
                continue;
            }
            const double t_acc = pt.velocity[i] / pt.acceleration[i];
            if (!near(pt.velocity[i] * (pt.time - t_acc), d)) {
                return false;
            }
        }
        return true;
    };

    testOk1(near(steps[0].time, 2.0 / v + v / a));
    testOk1(near(steps[0].velocity[0], v));
    testOk1(near(steps[0].velocity[1], v / 2.0));
    testOk1(near(steps[0].velocity[0] / steps[0].acceleration[0],
                 steps[0].velocity[1] / steps[0].acceleration[1]));
    testOk1(covers(steps[0], step_waypoints[0]));

    testOk1(near(steps[1].time, 2.0 * std::sqrt(0.05 / a)));
    testOk1(steps[1].velocity[0] < v);
    testOk1(covers(steps[1], step_waypoints[1]));

    testOk1(steps[2].time == 0.0);
    testOk1(covers(steps[2], step_waypoints[2]));
    testOk1(steps[2].position == step_waypoints[3]);

    return testDone();
}
