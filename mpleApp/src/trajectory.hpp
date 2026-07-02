#pragma once
#include <vector>
#include <string>

struct Waypoint {
    enum Space { JOINT, CARTESIAN };
    Space space;
    std::vector<double> values;
    double speed = 0.0;
    double accel = 0.0;
};

struct Trajectory {
    std::string id;
    std::vector<Waypoint> waypoints;
    bool validated = false;
};
