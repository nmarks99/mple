#include <epicsGuard.h>
#include <epicsThread.h>
#include <epicsUnitTest.h>
#include <testMain.h>
#include <iostream>

#include <trajectory.hpp>

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
        testOk1(p == 1.0);
    }

    return testDone();
}
