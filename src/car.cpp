//
// Created by sebas on 15/12/2019.
//

#include "car.h"
#include "constants.h"

#include <iostream>
#include <algorithm>
#include <omp.h>

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
    double t = num_time_steps * time_step_size;
    double t_v_max = (SPEED_LIMIT_LANE - v_abs_) / MAX_ACC_VEHICLES;
    double t_v_min = -v_abs_ / MAX_DEC_VEHICLES;
    double furthest_s, closest_s;

    if (t < t_v_max) {
        furthest_s = s_ + v_abs_ * t + 0.5 * MAX_ACC_VEHICLES * std::pow(t, 2.0);
    } else {
        furthest_s = s_ + v_abs_ * t_v_max + 0.5 * MAX_ACC_VEHICLES * std::pow(t_v_max, 2.0) + (t - t_v_max) * 80;
    }

    if (t < t_v_min) {
        closest_s = s_ + v_abs_ * t + 0.5 * MAX_DEC_VEHICLES * std::pow(t, 2.0);
    } else {
        closest_s = s_ + v_abs_ * t_v_min + 0.5 * MAX_DEC_VEHICLES * std::pow(t_v_min, 2.0);
    }

    return std::make_tuple(closest_s, furthest_s);
}

std::tuple<Lane, bool>
Car::behaviourPlanner(size_t const &prev_size, std::vector <std::vector<double>> const &sensor_fusion) {
    double max_add_on_s = 0.0;
    double current_vel = v_abs_;
    for (size_t i = 0; i < NUM_TIME_STEPS - prev_size; ++i) {
        current_vel += v_abs_ + MAX_ABS_ACC_EGO_VEHICLE;
        max_add_on_s += current_vel * TIME_STEP_SIZE;
    }

    double safety_distance =
            0.55 * v_abs_ * 3.6;

    /* determine current traffic condition based on predictions */
    bool car_ahead = false;
    bool car_ahead_dangerous = false;
    bool could_change_left = true;
    bool could_change_right = true;
    bool car_in_front_left = false;
    bool car_in_front_right = false;
    double car_in_front_left_dist = std::numeric_limits<double>::max();
    double car_in_front_right_dist = std::numeric_limits<double>::max();
    double car_ahead_dist = std::numeric_limits<double>::max();

    if (this->get_current_lane() == left) {
        could_change_left = false;
    } else if (this->get_current_lane() == right) {
        could_change_right = false;
    }

    #pragma omp parallel for schedule(guided) num_threads(OPEN_MP_NUM_THREADS) \
                    reduction(&&, could_change_left) reduction(&&, could_change_right) reduction(||, car_ahead_dangerous) \
                    reduction(||, car_ahead) reduction(||, car_in_front_right) reduction(||, car_in_front_left) \
                    reduction(min, car_ahead_dist) reduction(min, car_in_front_right_dist) \
                    reduction(min, car_in_front_left_dist)
    for (auto const &elem : sensor_fusion) {

        Car new_car(elem[5], elem[3], elem[6], elem[4]);
        new_car.determineLane();

        if (new_car.get_current_lane() == unknown) {
            continue;
        }

        auto[closest_s, furthest_s] = new_car.predictFutureSates(NUM_TIME_STEPS);

        bool dangerous = (closest_s - (safety_distance + max_add_on_s)) < this->get_s() &&
                         this->get_s() < (furthest_s + (safety_distance - max_add_on_s));

        if (new_car.get_current_lane() == this->get_current_lane()) {
            if (this->get_s() < new_car.get_s()) {
                if (car_ahead) {
                    car_ahead_dist = std::min(car_ahead_dist, new_car.get_s() - this->get_s());
                }
                car_ahead = true;
                car_ahead_dist = new_car.get_s() - this->get_s();
                if (dangerous && (v_abs_ + SPEED_BUFFER) > new_car.get_v_abs()) {
                    car_ahead_dangerous = true;
                }
            }
        } else {
            switch (this->get_current_lane()) {
                case middle: {
                    if (new_car.get_current_lane() == left) {
                        if (new_car.get_s() > this->get_s()) {
                            if (car_in_front_left) {
                                car_in_front_left_dist = std::min(car_in_front_left_dist,
                                                                  new_car.get_s() - this->get_s());
                            } else {
                                car_in_front_left_dist = new_car.get_s() - this->get_s();
                                car_in_front_left = true;
                            }
                        }

                        if (dangerous) {
                            could_change_left = false;
                        }
                    } else if (new_car.get_current_lane() == right) {
                        if (new_car.get_s() > this->get_s()) {
                            if (car_in_front_right) {
                                car_in_front_right_dist = std::min(car_in_front_right_dist,
                                                                   new_car.get_s() - this->get_s());
                            } else {
                                car_in_front_right_dist = new_car.get_s() - this->get_s();
                                car_in_front_right = true;
                            }
                        }
                        if (dangerous) {
                            could_change_right = false;
                        }
                    }
                    break;
                }
                case left: {
                    if (new_car.get_current_lane() == middle) {
                        if (car_in_front_right) {
                            car_in_front_right_dist = std::min(car_in_front_right_dist,
                                                               new_car.get_s() - this->get_s());
                        } else {
                            car_in_front_right_dist = new_car.get_s() - this->get_s();
                            car_in_front_right = true;
                        }
                        if (dangerous) {
                            could_change_right = false;
                        }
                    }
                    break;
                }
                case right: {
                    if (new_car.get_current_lane() == middle) {
                        if (new_car.get_s() > this->get_s()) {
                            if (car_in_front_left) {
                                car_in_front_left_dist = std::min(car_in_front_left_dist,
                                                                  new_car.get_s() - this->get_s());
                            } else {
                                car_in_front_left_dist = new_car.get_s() - this->get_s();
                                car_in_front_left = true;
                            }
                        }
                        if (dangerous) {
                            could_change_left = false;
                        }
                    }
                    break;
                }
                default: {
                    break;
                }
            }
        }
    }


    Lane next_lane = this->get_current_lane();
    bool changed_lane = false;

    // change lane if a car is directly in front in the current lane and it is safe to change
    if (car_ahead_dangerous && (could_change_left || could_change_right)) {
        next_lane = static_cast<Lane>(could_change_left ? int(next_lane) - 1 :
                                      int(next_lane) + 1);
        changed_lane = true;
    }

    // change lane if other lane has more free space in front and it is safe to change
    if (!changed_lane && car_ahead) {
        int i = 0;
        if (could_change_left && could_change_right) {
            if (!car_in_front_left) {
                i = -1;
            } else if (!car_in_front_right) {
                i = 1;
            } else if (std::max(car_in_front_left, car_in_front_right) >
                       (car_ahead_dist + DISTANCE_BUFFER)) {
                if (car_in_front_left > car_in_front_right) {
                    i = -1;
                } else {
                    i = 1;
                }
            }
        } else if (could_change_left) {
            if (!car_in_front_left || (car_in_front_left > (car_ahead_dist + DISTANCE_BUFFER))) {
                i = -1;
            }
        } else if (could_change_right) {
            if (!car_in_front_right || (car_in_front_right > (car_ahead_dist + DISTANCE_BUFFER))) {
                i = 1;
            }
        }

        if (i != 0) {
            next_lane = static_cast<Lane>(int(next_lane) + i);
            changed_lane = true;
        }
    }

    return std::make_tuple(next_lane, car_ahead_dangerous);;
}