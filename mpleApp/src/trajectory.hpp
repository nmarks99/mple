#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <vector>

class GeometricPath {
  public:
    GeometricPath(std::vector<std::vector<double>> waypoints) : waypoints_(waypoints) {
        assert(waypoints_.size() >= 2);
        q_size_ = waypoints[0].size();

        d_cumulative_.push_back(0.0);
        for (size_t i = 0; i < waypoints.size() - 1; i++) {
            Eigen::Map<const Eigen::VectorXd> v1(waypoints[i].data(), q_size_);
            Eigen::Map<const Eigen::VectorXd> v2(waypoints[i + 1].data(), q_size_);
            auto d = (v2 - v1).norm();
            d_segments_.push_back(d);
            d_cumulative_.push_back(d + d_cumulative_.back());
        }
        length_ = d_cumulative_.back();
    }

    double length() const { return length_; }

    std::vector<double> sample(double x) const {
        // Handle boundary conditions explicitly
        if (x <= 0.0) {
            return waypoints_.front();
        }
        if (x >= length_) {
            return waypoints_.back();
        }

        std::vector<double> q_out(6);

        // Find the two waypoints which enclose the given x
        auto it = std::upper_bound(d_cumulative_.begin(), d_cumulative_.end(), x);
        size_t index_high = std::distance(d_cumulative_.begin(), it);
        size_t index_low = index_high - 1;
        auto q0 = waypoints_.at(index_low);
        auto q1 = waypoints_.at(index_high);

        // Prevent possible division by zero
        const double dseg = d_segments_.at(index_low);
        const double dcum = d_cumulative_.at(index_low);
        if (dseg == 0.0) {
            // indentical consecutive waypoints. Return either one:
            return q0;
        }

        // Interpolate to find a q at the given distance x from the start of the path
        double alpha = (x - dcum) / dseg;
        for (size_t i = 0; i < q0.size(); ++i) {
            q_out[i] = (1 - alpha) * q0[i] + alpha * q1[i];
        }

        return q_out;
    }

  private:
    size_t q_size_;
    double length_ = 0.0;
    std::vector<std::vector<double>> waypoints_;
    std::vector<double> d_segments_;
    std::vector<double> d_cumulative_;
};

inline std::vector<std::vector<double>> create_trajectory(const GeometricPath& path, double vmax, double amax,
                                                          double dt) {

    std::vector<std::vector<double>> traj;

    const double L = path.length();
    const double Tv = (15.0 / 8.0) * (L / vmax);
    const double Ta = std::sqrt((10 / std::sqrt(3)) * (L / amax));
    double Tf = std::max(Tv, Ta);

    double t = 0.0;
    while (t <= Tf) {
        const double u = t / Tf;
        const double s = 10 * std::pow(u, 3) - 15 * std::pow(u, 4) + 6 * std::pow(u, 5);
        const double x = L * s;
        auto q = path.sample(x);
        traj.push_back(q);
        t += dt;
    }

    return traj;
}
