//
// Created by sebas on 15/12/2019.
//

#include "car.h"
#include "trajectory.h"

#include <iostream>
#include <algorithm>

void Car::determineLane() {
    if (d_ > 0 && d_ < 4) {
        current_lane_ = left;
    } else if (d_ > 4 && d_ < 8) {
        current_lane_ = middle;
    } else if (d_ > 8 && d_ < 12) {
        current_lane_ = right;
    } else {
        current_lane_ = other;
    }
}

std::tuple<double, double> Car::predictFutureSates(size_t const &num_time_steps, double time_step_size) {
    double max_acc = 10.0;
    double max_dec = -10.0;
    double t = num_time_steps * time_step_size;
    double furthest_s = s_ + v_abs_ * t + 0.5 * max_acc * std::pow(t, 2.0);
    double closest_s = s_ + v_abs_ * t + 0.5 * max_dec * std::pow(t, 2.0);


    return std::make_tuple(closest_s, furthest_s);
}

std::tuple <std::vector<double>, std::vector<double>>
Car::generateTrajectory(size_t const &num_time_steps, double const &target_s, double const &target_d_) {

    double duration = num_time_steps * 0.02;
    double target_d = d_;
    double target_d_d = 0;
    double target_d_dd = 0;
    double target_s_d = std::min(s_d_ + 2.0 * duration, 50.0);
    double target_s_dd = 0;
    double target_s_ = s_ + (s_d_ + target_s_d) / (2 * duration);
    std::vector<double> start_s{s_, s_d_, s_dd_};
    std::vector<double> end_s{target_s_, target_s_d, target_s_dd};
    auto param_s = JMT(start_s, end_s, duration);
    std::vector<double> start_d{d_, d_d_, d_dd_};
    std::vector<double> end_d{target_d_, target_d_d, target_d_dd};
    auto param_d = JMT(start_d, end_d, duration);
    std::vector<double> s_traj;
    std::vector<double> d_traj;

    // populate s and t trajectories at each time step
    for (int i = 0; i < num_time_steps; i++) {
        double t = i * duration/num_time_steps;
        double s_val = 0, d_val = 0;
        for (int j = 0; j < param_s.size(); j++) {
            s_val += param_s[j] * pow(t, j);
            d_val += param_d[j] * pow(t, j);
        }
        s_traj.push_back(s_val);
        d_traj.push_back(d_val);
    }


    return std::make_pair(s_traj, d_traj);
}