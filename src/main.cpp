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

#include "car.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

const double time_step_size = 0.02;

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
                auto j = json::parse(s);

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

                    json msgJson;

                    double pos_x, pos_y;
                    double angle;
                    double s, s_d, s_dd;
                    double d, d_d, d_dd;

                    // use default values if not enough previous path points
                    if (previous_path_x.size() < 4) {
                        pos_x = car_x;
                        pos_y = car_y;
                        angle = deg2rad(car_yaw);
                        s = car_s;
                        d = car_d;
                        s_d = car_speed;
                        d_d = 0;
                        s_dd = 0;
                        d_dd = 0;
                    } else {
                        std::cout << "to be implemented" << std::endl;

                        // calculate ego vehicle parameters based on the last four coordinates

                        double pos_x_1 = previous_path_x[previous_path_x.size() - 1];
                        double pos_x_2 = previous_path_x[previous_path_x.size() - 2];
                        double pos_x_3 = previous_path_x[previous_path_x.size() - 3];
                        double pos_x_4 = previous_path_x[previous_path_x.size() - 4];

                        double pos_y_1 = previous_path_y[previous_path_x.size() - 1];
                        double pos_y_2 = previous_path_y[previous_path_x.size() - 2];
                        double pos_y_3 = previous_path_y[previous_path_x.size() - 3];
                        double pos_y_4 = previous_path_y[previous_path_x.size() - 4];

                        auto coords_1 = getFrenet(pos_x_1, pos_y_1, atan2(pos_y_1-pos_y_2,pos_x_1-pos_x_2), map_waypoints_x, map_waypoints_y);
                        auto coords_2 = getFrenet(pos_x_2, pos_y_2, atan2(pos_y_2-pos_y_3,pos_x_2-pos_x_3), map_waypoints_x, map_waypoints_y);
                        auto coords_3 = getFrenet(pos_x_3, pos_y_3, atan2(pos_y_3-pos_y_4,pos_x_3-pos_x_4), map_waypoints_x, map_waypoints_y);

                        s_d = (coords_1[0] - coords_2[0]) / time_step_size;
                        s_dd = (s_d - (coords_2[0] - coords_3[0]) / time_step_size) / (2.0 * time_step_size);

                        d_d = (coords_1[1] - coords_2[1]) / time_step_size;
                        d_dd = (d_d - (coords_2[1] - coords_3[1]) / time_step_size) / (2.0 * time_step_size);
                    }

                    Car ego_car(s, s_d, s_dd, d, d_d, d_dd);


                    std::vector<Car> other_traffic_participants;
                    other_traffic_participants.reserve(sensor_fusion.size());
                    for (auto const &elem : sensor_fusion) {
                        Car new_car(elem[5], elem[3], elem[6], elem[4]);
                        new_car.determineLane();
                        auto [ closest_s, furthest_s ] = new_car.predictFutureSates(previous_path_x.size());
                        other_traffic_participants.emplace_back(std::move(new_car));
                    }


                    std::vector<double> next_x_vals;
                    std::vector<double> next_y_vals;

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


