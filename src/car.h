//
// Created by sebas on 15/12/2019.
//

#ifndef CARND_PATH_PLANNING_PROJECT_CAR_H
#define CARND_PATH_PLANNING_PROJECT_CAR_H

#include "math.h"
#include <tuple>
#include <vector>

enum Lane {
    left = 0, middle = 1, right = 2, other = 3, unknown = -1
};


class Car {

public:

    Car() = default;

    Car(double s, double v_x, double d, double v_y) {
        s_ = s;
        d_ = d;
        v_abs_ = std::sqrt(v_x * v_x + v_y * v_y);
        current_lane_ = unknown;
    }

    Car(double s, double s_d, double d, double d_d, Lane current_lane) {
        s_ = s;
        s_d_ = s_d;
        d_ = d;
        d_d_ = d_d;
        current_lane_ = current_lane;
    }

    ~Car() {}

    /* setter and getter */

    double get_s() const { return s_; }

    double get_s_d() const { return s_d_; }

    double get_d() const { return s_; }

    double get_d_d() const { return d_d_; }

    Lane get_current_lane() const { return current_lane_; }

    double get_v_abs() const { return v_abs_; }

    /* member functions */
    void determineLane();

    std::tuple<double, double> predictFutureSates(size_t const &num_time_steps, double time_step_size = 0.02);


private:

    double s_;
    double s_d_;

    double d_;
    double d_d_;

    double v_abs_;
    Lane current_lane_;
};


#endif //CARND_PATH_PLANNING_PROJECT_CAR_H
