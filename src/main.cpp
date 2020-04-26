#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include "map.h"
#include "path_planner.h"
#include "prediction.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
    uWS::Hub h;

    Map map;

    bool prediction_initialized = false;
    int lane = 1;
    double ref_vel = 0.0;

    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";
    map.read_map(map_file_);

    std::vector<double> map_waypoints_x = map.map_waypoints_x;
    std::vector<double> map_waypoints_y = map.map_waypoints_y;
    std::vector<double> map_waypoints_s = map.map_waypoints_s;
    std::vector<double> map_waypoints_dx = map.map_waypoints_dx;
    std::vector<double> map_waypoints_dy = map.map_waypoints_dy;

    h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
        &map_waypoints_dx, &map_waypoints_dy, &prediction_initialized, &lane, &ref_vel]
        (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
            uWS::OpCode opCode) {
                // "42" at the start of the message means there's a websocket message event.
                // The 4 signifies a websocket message
                // The 2 signifies a websocket event
                if (length && length > 2 && data[0] == '4' && data[1] == '2') {

                    Prediction obj_prediction;
                    PathPlanner path_planner;

                    auto s = hasData(data);

                    if (s != "") {
                        auto j = json::parse(s);

                        string event = j[0].get<string>();

                        if (event == "telemetry") {
                            // j[1] is the data JSON object

                            // Main car's localization Data
                            path_planner.car_parameters_t.car_x = j[1]["x"];
                            path_planner.car_parameters_t.car_y = j[1]["y"];
                            double car_s = j[1]["s"];
                            path_planner.car_parameters_t.car_d = j[1]["d"];
                            path_planner.car_parameters_t.car_yaw = j[1]["yaw"];
                            path_planner.car_parameters_t.car_speed = j[1]["speed"];
                            path_planner.car_parameters_t.car_s = car_s;

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
                            int prev_size = previous_path_x.size();
                            vector<double> next_x_vals;
                            vector<double> next_y_vals;

                            obj_prediction.prediction_init();                            
                            obj_prediction.prediction_doit(prev_size, end_path_s, sensor_fusion, lane, ref_vel);

                            path_planner.path_planner_init(lane, ref_vel);
                            path_planner.path_planner(path_planner.car_parameters_t, previous_path_x, previous_path_y, map_waypoints_s, map_waypoints_x, map_waypoints_y,
                                next_x_vals, next_y_vals);

                            msgJson["next_x"] = next_x_vals;
                            msgJson["next_y"] = next_y_vals;

                            auto msg = "42[\"control\"," + msgJson.dump() + "]";

                            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                        }  // end "telemetry" if
                    }
                    else {
                        // Manual driving
                        std::string msg = "42[\"manual\",{}]";
                        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                    }
                }  // end websocket if
        }); // end h.onMessage

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
        });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
        char *message, size_t length) {
            ws.close();
            std::cout << "Disconnected" << std::endl;
        });

    int port = 4567;
    if (h.listen("127.0.0.1", port))
    {
        std::cout << "Listening to port " << port << std::endl;
    }
    else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }

    h.run();
}