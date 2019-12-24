//
// Created by sebas on 15/12/2019.
//

#include "car.h"
#include "constants.h"
#include "spline.h"

#include <iostream>
#include <algorithm>
#include <omp.h>
#include <math.h>

// Transform from Frenet s,d coordinates to Cartesian x,y
std::vector<double> getXY(double s, double d, const std::vector<double> &maps_s,
                          const std::vector<double> &maps_x,
                          const std::vector<double> &maps_y) {
    int prev_wp = -1;

    while (s > maps_s[prev_wp + 1] && (prev_wp < (int) (maps_s.size() - 1))) {
        ++prev_wp;
    }

    int wp2 = (prev_wp + 1) % maps_x.size();

    double heading = atan2((maps_y[wp2] - maps_y[prev_wp]),
                           (maps_x[wp2] - maps_x[prev_wp]));
    // the x,y,s along the segment
    double seg_s = (s - maps_s[prev_wp]);

    double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
    double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

    double perp_heading = heading - M_PI / 2;

    double x = seg_x + d * cos(perp_heading);
    double y = seg_y + d * sin(perp_heading);

    return {x, y};
}

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

    return std::make_tuple(next_lane, car_ahead_dangerous);
}

std::tuple <std::vector<double>, std::vector<double>>
Car::generateTrajectory(Lane const &next_lane, bool const &car_ahead_dangerous,
                        std::vector<double> const &previous_path_x, std::vector<double> const &previous_path_y,
                        std::vector<double> &ptsx, std::vector<double> &ptsy,
                        std::vector<double> const &map_waypoints_x,
                        std::vector<double> const &map_waypoints_y,
                        std::vector<double> const &map_waypoints_s) {

    size_t prev_size = previous_path_x.size();
    double pos_x = ptsx.back();
    double pos_y = ptsy.back();

    bool changed_lane = next_lane != this->get_current_lane();

    // adjust speed
    double speed_diff = 0;
    if (car_ahead_dangerous && !changed_lane) {
        speed_diff -= MAX_ABS_ACC_EGO_VEHICLE;
    } else {
        if (this->get_v_abs() < SPEED_LIMIT) {
            speed_diff += MAX_ABS_ACC_EGO_VEHICLE;
        }
    }

    // Setting up target points in the future.
    std::vector<double> next_wp0 = getXY(this->get_s() + 30, 2 + 4 * int(next_lane), map_waypoints_s,
                                         map_waypoints_x,
                                         map_waypoints_y);
    std::vector<double> next_wp1 = getXY(this->get_s() + 40, 2 + 4 * int(next_lane), map_waypoints_s,
                                         map_waypoints_x,
                                         map_waypoints_y);
    std::vector<double> next_wp2 = getXY(this->get_s() + 50, 2 + 4 * int(next_lane), map_waypoints_s,
                                         map_waypoints_x,
                                         map_waypoints_y);
    std::vector<double> next_wp3 = getXY(this->get_s() + 60, 2 + 4 * int(next_lane), map_waypoints_s,
                                         map_waypoints_x,
                                         map_waypoints_y);

    ptsx.emplace_back(next_wp0[0]);
    ptsx.emplace_back(next_wp1[0]);
    ptsx.emplace_back(next_wp2[0]);
    ptsx.emplace_back(next_wp3[0]);

    ptsy.emplace_back(next_wp0[1]);
    ptsy.emplace_back(next_wp1[1]);
    ptsy.emplace_back(next_wp2[1]);
    ptsy.emplace_back(next_wp3[1]);

    // convert coordinates to local car coordinates.
    for (int i = 0; i < ptsx.size(); i++) {
        double shift_x = ptsx[i] - pos_x;
        double shift_y = ptsy[i] - pos_y;

        ptsx[i] = shift_x * cos(0 - this->get_angle()) - shift_y * sin(0 - this->get_angle());
        ptsy[i] = shift_x * sin(0 - this->get_angle()) + shift_y * cos(0 - this->get_angle());
    }


    std::vector<double> next_x_vals;
    next_x_vals.reserve(NUM_TIME_STEPS);

    std::vector<double> next_y_vals;
    next_y_vals.reserve(NUM_TIME_STEPS);

    // use path points from previous path for continuity
    next_x_vals.insert(std::end(next_x_vals), std::begin(previous_path_x), std::end(previous_path_x));
    next_y_vals.insert(std::end(next_y_vals), std::begin(previous_path_y), std::end(previous_path_y));


    // create the spline.
    tk::spline spline_;
    spline_.set_points(ptsx, ptsy);

    // Calculate distance y position on 30 m ahead.
    double target_x = 30.0;
    double target_y = spline_(target_x);
    double target_dist = sqrt(target_x * target_x + target_y * target_y);

    double x_add_on = 0;

    double x_ref, y_ref, x_point, y_point, N;
    double ref_vel = this->get_v_abs();

    for (int i = 1; i < NUM_TIME_STEPS - prev_size; i++) {

        // perform acceleration or deceleration
        ref_vel += speed_diff;

        // check if speed is in range [0, 50 mph]
        ref_vel = std::max(std::min(ref_vel, SPEED_LIMIT), 0.0);

        // get x- and y-value in local coordinate system
        N = target_dist / (TIME_STEP_SIZE * ref_vel);
        x_point = x_add_on + target_x / N;
        y_point = spline_(x_point);

        // save current x-distance for next iteration
        x_add_on = x_point;

        // convert back to global coordinates
        x_ref = x_point;
        y_ref = y_point;
        x_point = x_ref * cos(this->get_angle()) - y_ref * sin(this->get_angle());
        y_point = x_ref * sin(this->get_angle()) + y_ref * cos(this->get_angle());
        x_point += pos_x;
        y_point += pos_y;

        next_x_vals.emplace_back(x_point);
        next_y_vals.emplace_back(y_point);
    }

    return std::make_tuple(next_x_vals, next_y_vals);
}