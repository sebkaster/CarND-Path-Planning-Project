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