#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include <omp.h>

#include "car.h"

/* define constants */
const double TIME_STEP_SIZE = 0.02;
const double NUM_TIME_STEPS = 25;
const double SPEED_LIMIT = 22; // m/s
const double MAX_ACC = 5.0 * TIME_STEP_SIZE; // m/s^2
const double SAFETY_DISTANCE = 20;
const double SPEED_BUFFER = 5.0;
const unsigned NUM_THREADS = 4;


int main() {
    uWS::Hub h;
    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;

    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";
    // The max s value before wrapping around the track back to 0
    double max_s = 6945.554;

    std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

    string line;
    while (getline(in_map_, line)) {
        std::istringstream iss(line);
        double x;
        double y;
        float s;
        float d_x;
        float d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        map_waypoints_x.push_back(x);
        map_waypoints_y.push_back(y);
        map_waypoints_s.push_back(s);
        map_waypoints_dx.push_back(d_x);
        map_waypoints_dy.push_back(d_y);
    }

    // Car's lane. Starting at middle lane.
    int lane = 1;

    // Reference velocity.
    double ref_vel = 0.0; // mph

    h.onMessage([&ref_vel, &lane, &map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
                        &map_waypoints_dx, &map_waypoints_dy]
                        (uWS::WebSocket <uWS::SERVER> ws, char *data, size_t length,
                         uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {

            auto s = hasData(data);

            if (s != "") {
                auto j = nlohmann::json::parse(s);

                string event = j[0].get<string>();

                if (event == "telemetry") {
                    // j[1] is the data JSON object

                    // Main car's localization Data
                    double car_x = j[1]["x"];
                    double car_y = j[1]["y"];
                    double car_s = j[1]["s"];
                    double car_d = j[1]["d"];
                    double car_yaw = j[1]["yaw"];
                    double car_speed = j[1]["speed"];

                    // Previous path data given to the Planner
                    auto previous_path_x = j[1]["previous_path_x"];
                    auto previous_path_y = j[1]["previous_path_y"];


                    // Previous path's end s and d values
                    double end_path_s = j[1]["end_path_s"];
                    double end_path_d = j[1]["end_path_d"];

                    // Sensor Fusion Data, a list of all other cars on the same side
                    //   of the road.
                    auto sensor_fusion = j[1]["sensor_fusion"];

                    nlohmann::json msgJson;

                    size_t prev_size = previous_path_x.size();

                    // Preventing collisions
                    if (prev_size > 0) {
                        car_s = end_path_s;
                    }


                    std::vector<double> ptsx;
                    ptsx.reserve(5);
                    std::vector<double> ptsy;
                    ptsy.reserve(5);

                    double pos_x, pos_y;
                    double angle;
                    double s, s_d, s_dd;
                    double d, d_d, d_dd;

                    // Do I have have previous points
                    if (prev_size < 2) {
                        // use two points to make the path tangent
                        double prev_car_x = car_x - cos(car_yaw);
                        double prev_car_y = car_y - sin(car_yaw);

                        ptsx.emplace_back(prev_car_x);
                        ptsx.emplace_back(car_x);

                        ptsy.emplace_back(prev_car_y);
                        ptsy.emplace_back(car_y);

                        angle = deg2rad(car_yaw);
                        s = car_s;
                        d = car_d;
                        s_d = car_speed;
                        d_d = 0;
                    } else {
                        // Use the last two points

                        double pos_x_1 = previous_path_x[prev_size - 1];
                        double pos_x_2 = previous_path_x[prev_size - 2];
                        double pos_x_3 = previous_path_x[prev_size - 3];

                        double pos_y_1 = previous_path_y[prev_size - 1];
                        double pos_y_2 = previous_path_y[prev_size - 2];
                        double pos_y_3 = previous_path_y[prev_size - 3];

                        angle = atan2(pos_y_1 - pos_y_2, pos_x_1 - pos_x_2);
                        auto coords_1 = getFrenet(pos_x_1, pos_y_1, angle, map_waypoints_x, map_waypoints_y);
                        auto coords_2 = getFrenet(pos_x_2, pos_y_2, atan2(pos_y_2 - pos_y_3, pos_x_2 - pos_x_3),
                                                  map_waypoints_x, map_waypoints_y);

                        car_speed = distance(pos_x_1, pos_y_1, pos_x_2, pos_y_2) / TIME_STEP_SIZE;

                        s = coords_1[0];
                        s_d = (coords_1[0] - coords_2[0]) / TIME_STEP_SIZE;

                        d = coords_1[1];
                        d_d = (coords_1[1] - coords_2[1]) / TIME_STEP_SIZE;

                        ptsx.emplace_back(pos_x_2);
                        ptsx.emplace_back(pos_x_1);

                        ptsy.emplace_back(pos_y_2);
                        ptsy.emplace_back(pos_y_1);
                    }

                    pos_x = ptsx.back();
                    pos_y = ptsy.back();

                    Car ego_car(s, s_d, d, d_d, middle);
                    ego_car.determineLane();

                    double max_add_on_s = 0.0;
                    double current_vel = car_speed;
                    for (size_t i = 0; i < NUM_TIME_STEPS - prev_size; ++i) {
                        current_vel += car_speed + MAX_ACC;
                        max_add_on_s += current_vel * TIME_STEP_SIZE;
                    }

                    double safety_distance =
                            0.55 * car_speed * 3.6;

                    bool car_ahead = false;
                    bool car_ahead_dangerous = false;
                    bool could_change_left = true;
                    bool could_change_right = true;
                    bool car_in_front_left = false;
                    bool car_in_front_right = false;

                    if (ego_car.get_current_lane() == left) {
                        could_change_left = false;
                    } else if (ego_car.get_current_lane() == right) {
                        could_change_right = false;
                    }

                    #pragma omp parallel for schedule(guided) num_threads(OPEN_MP_NUM_THREADS) \
                    reduction(&&, could_change_left) reduction(&&, could_change_right) reduction(||, car_ahead_dangerous) \
                    reduction(||, car_ahead) reduction(||, car_in_front_right) reduction(||, car_in_front_left)
                    for (auto const &elem : sensor_fusion) {
                        Car new_car(elem[5], elem[3], elem[6], elem[4]);
                        new_car.determineLane();

                        if (new_car.get_current_lane() == unknown) {
                            continue;
                        }
                        auto[closest_s, furthest_s] = new_car.predictFutureSates(NUM_TIME_STEPS);

                        bool dangerous = (closest_s - (safety_distance + max_add_on_s)) < ego_car.get_s() &&
                                         ego_car.get_s() < (furthest_s + (safety_distance - max_add_on_s));

                        if (new_car.get_current_lane() == ego_car.get_current_lane()) {
                            if (ego_car.get_s() < new_car.get_s() && dangerous &&
                                (car_speed + SPEED_BUFFER) > new_car.get_v_abs()) {
                                car_ahead_dangerous = true;
                            }
                        } else {
                            switch (ego_car.get_current_lane()) {
                                case middle: {
                                    if (new_car.get_current_lane() == left && dangerous) {
                                        could_change_left = false;
                                    } else if (new_car.get_current_lane() == right && dangerous) {
                                        could_change_right = false;
                                    }
                                    break;
                                }
                                case left: {
                                    if (new_car.get_current_lane() == middle && dangerous) {
                                        could_change_right = false;
                                    }
                                    break;
                                }
                                case right: {
                                    if (new_car.get_current_lane() == middle && dangerous) {
                                        could_change_left = false;
                                    }
                                    break;
                                }
                                default: {
                                    break;
                                }
                            }
                        }
                    }


                    Lane next_lane = ego_car.get_current_lane();
                    bool changed_lane = false;
                    if (car_ahead_dangerous && (could_change_left || could_change_right)) {

                        next_lane = static_cast<Lane>(could_change_left ? int(next_lane) - 1 :
                                                      int(next_lane) + 1);
                        changed_lane = true;
                    }

                    double speed_diff = 0;
                    if (car_ahead_dangerous && !changed_lane) {
                        speed_diff -= MAX_ACC;
                    } else {
                        if (ref_vel < SPEED_LIMIT) {
                            speed_diff += MAX_ACC;
                        }
                    }

                    // Setting up target points in the future.
                    std::vector<double> next_wp0 = getXY(car_s + 30, 2 + 4 * int(next_lane), map_waypoints_s,
                                                    map_waypoints_x,
                                                    map_waypoints_y);
                    std::vector<double> next_wp1 = getXY(car_s + 60, 2 + 4 * int(next_lane), map_waypoints_s,
                                                    map_waypoints_x,
                                                    map_waypoints_y);
                    std::vector<double> next_wp2 = getXY(car_s + 90, 2 + 4 * int(next_lane), map_waypoints_s,
                                                    map_waypoints_x,
                                                    map_waypoints_y);

                    ptsx.emplace_back(next_wp0[0]);
                    ptsx.emplace_back(next_wp1[0]);
                    ptsx.emplace_back(next_wp2[0]);

                    ptsy.emplace_back(next_wp0[1]);
                    ptsy.emplace_back(next_wp1[1]);
                    ptsy.emplace_back(next_wp2[1]);

                    // Making coordinates to local car coordinates.
                    for (int i = 0; i < ptsx.size(); i++) {
                        double shift_x = ptsx[i] - pos_x;
                        double shift_y = ptsy[i] - pos_y;

                        ptsx[i] = shift_x * cos(0 - angle) - shift_y * sin(0 - angle);
                        ptsy[i] = shift_x * sin(0 - angle) + shift_y * cos(0 - angle);
                    }

                    // Create the spline.
                    tk::spline spline_;
                    spline_.set_points(ptsx, ptsy);

                    // Output path points from previous path for continuity.
                    std::vector<double> next_x_vals;
                    next_x_vals.reserve(NUM_TIME_STEPS);

                    std::vector<double> next_y_vals;
                    next_y_vals.reserve(NUM_TIME_STEPS);

                    next_x_vals.insert(std::end(next_x_vals), std::begin(previous_path_x), std::end(previous_path_x));
                    next_y_vals.insert(std::end(next_y_vals), std::begin(previous_path_y), std::end(previous_path_y));

                    // Calculate distance y position on 30 m ahead.
                    double target_x = 30.0;
                    double target_y = spline_(target_x);
                    double target_dist = sqrt(target_x * target_x + target_y * target_y);

                    double x_add_on = 0;

                    for (int i = 1; i < NUM_TIME_STEPS - prev_size; i++) {
                        ref_vel += speed_diff;
                        ref_vel = std::max(std::min(ref_vel, SPEED_LIMIT), 0.0);
                        double N = target_dist / (0.02 * ref_vel);
                        double x_point = x_add_on + target_x / N;
                        double y_point = spline_(x_point);

                        x_add_on = x_point;

                        double x_ref = x_point;
                        double y_ref = y_point;

                        x_point = x_ref * cos(angle) - y_ref * sin(angle);
                        y_point = x_ref * sin(angle) + y_ref * cos(angle);

                        x_point += pos_x;
                        y_point += pos_y;

                        next_x_vals.emplace_back(x_point);
                        next_y_vals.emplace_back(y_point);
                    }

                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;

                    auto msg = "42[\"control\"," + msgJson.dump() + "]";

                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }  // end "telemetry" if
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }  // end websocket if
    }); // end h.onMessage

    h.onConnection([&h](uWS::WebSocket <uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket <uWS::SERVER> ws, int code,
                           char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (h.listen(port)) {
        std::cout << "Listening to port " << port << std::endl;
    } else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }

    h.run();
}


