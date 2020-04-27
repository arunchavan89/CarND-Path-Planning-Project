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

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
    uWS::Hub h;

    /* Default start lane number */
    int lane = 1;

    /* The vehicle prior velocity */
    double ref_vel = 0.0;

    /* The vehicle´s maximum velocity allowed in MPH */
    double max_speed_MPH = 49.0;

    /* The vehicle allowed to accelerate faster until this speed limit. */
    double max_speed_MPH_1 = 47.0;

    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;

    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";

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

    h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
        &map_waypoints_dx, &map_waypoints_dy, &lane, &max_speed_MPH, &ref_vel, &max_speed_MPH_1]
        (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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

                            /* The path points the vehicle would follow */
                            vector<double> next_x_vals;
                            vector<double> next_y_vals;

                            /* Acceleration rates */
                            double accleration_rate = 0.224;
                            double aggresive_accleration_rate = 1.0;

                            /* Braking decceleration when emergency */
                            double  emergency_braking = 15.0;
                            double  speed_car_ahead = 0.0;

                            /* Size of the previous path*/
                            int prev_size = previous_path_x.size();

                            if (prev_size > 0)
                            {
                                car_s = end_path_s;
                            }

                            bool car_lane_0 = false;
                            bool car_lane_1 = false;
                            bool car_lane_2 = false;
                            bool car_ahead = false;
                            bool emergency_brake = false;

                            for (int i = 0; i < sensor_fusion.size(); i++)
                            {
                                double d = sensor_fusion[i][6];
                                double vx = sensor_fusion[i][3];
                                double vy = sensor_fusion[i][4];
                                double check_speed = sqrt(vx * vx + vy * vy);
                                double check_car_s = sensor_fusion[i][5];

                                //This will help to predict the where the vehicle will be in future
                                check_car_s += ((double)prev_size * 0.02 * check_speed);

                                /* Check if any vehicle is ahead of the ego vehicle */
                                if ((d < (2 + 4 * lane + 2)) && (d > (2 + 4 * lane - 2)))
                                {
                                    if ((check_car_s > car_s) && (check_car_s - car_s) < 30)
                                    {
                                        car_ahead = true;
                                        speed_car_ahead = check_speed;
                                        if ((check_car_s > car_s) && (check_car_s - car_s) < 10)
                                        {
                                            emergency_brake = true;
                                        }
                                    }
                                }

                                /* Check if any vehicle present in lane 0*/
                                if ((d > 0) && (d < 4))
                                {
                                    if (((check_car_s > car_s) && (check_car_s - car_s) < 35) ||
                                        ((car_s > check_car_s) && (car_s - check_car_s) < 10))
                                    {
                                        car_lane_0 = true;
                                    }
                                }

                                /* Check if any vehicle present in lane 1*/
                                if ((d > 4) && (d < 8))
                                {
                                    if (((check_car_s > car_s) && (check_car_s - car_s) < 35) ||
                                        ((car_s > check_car_s) && (car_s - check_car_s) < 10))
                                    {
                                        car_lane_1 = true;
                                    }
                                }

                                /* Check if any vehicle present in lane 2*/
                                if ((d > 8) && (d < 12))
                                {
                                    if (((check_car_s > car_s) && (check_car_s - car_s) < 35) ||
                                        ((car_s > check_car_s) && (car_s - check_car_s) < 10))
                                    {
                                        car_lane_2 = true;
                                    }
                                }

                            }

                            /* If any vehicle is ahead of the ego car, take an appropriate decision. */
                            if (car_ahead)
                            {
                                if ((lane == 1) && !car_lane_0)
                                {
                                    lane--;
                                }
                                else if ((lane == 1) && !car_lane_2)
                                {
                                    lane++;
                                }
                                else if ((lane == 2) && !car_lane_1)
                                {
                                    lane--;
                                }
                                else if ((lane == 0) && !car_lane_1)
                                {
                                    lane++;
                                }
                                else
                                {
                                    if (emergency_brake)
                                    {
                                        /* Possibility of getting jerk but good for safety */
                                        std::cout << "Emergency Brake:" << std::endl;
                                        ref_vel -= emergency_braking;
                                    }
                                    else
                                    {
                                        if (ref_vel != speed_car_ahead)
                                        {
                                            ref_vel -= accleration_rate;
                                        }
                                    }
                                }
                            }
                            else if (ref_vel < max_speed_MPH)
                            {
                                if (ref_vel < max_speed_MPH_1)
                                {
                                    ref_vel += aggresive_accleration_rate;
                                }
                                if (ref_vel < max_speed_MPH)
                                {
                                    ref_vel += accleration_rate;
                                }
                            }

                            /* creating spline till next 30 meters */
                            double  target_x = 30.0;
                            double time_per_frame = 0.02;
                            std::vector<double> pts_x;
                            std::vector<double> pts_y;

                            double  ref_x = car_x;
                            double ref_y = car_y;
                            double  ref_yaw = deg2rad(car_yaw);

                            // If previous states are almost empty, use the car as a starting point
                            if (prev_size < 2)
                            {
                                //Use two points thats makes path tangent to the car
                                double prev_car_x = ref_x - cos(car_yaw);
                                double prev_car_y = ref_y - sin(car_yaw);

                                pts_x.push_back(prev_car_x);
                                pts_y.push_back(prev_car_y);

                                pts_x.push_back(ref_x);
                                pts_y.push_back(ref_y);

                            }
                            else
                            {
                                /* Redefine the reference point to the last point in the previous point list */
                                ref_x = previous_path_x[prev_size - 1];
                                ref_y = previous_path_y[prev_size - 1];

                                double ref_x_prev = previous_path_x[prev_size - 2];
                                double ref_y_prev = previous_path_y[prev_size - 2];

                                ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

                                pts_x.push_back(ref_x_prev);
                                pts_y.push_back(ref_y_prev);

                                pts_x.push_back(ref_x);
                                pts_y.push_back(ref_y);
                            }

                            // Setting up target points in the future.
                            std::vector<double> next_wp0 = getXY(car_s + 30, 2 + (4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
                            std::vector<double> next_wp1 = getXY(car_s + 60, 2 + (4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
                            std::vector<double> next_wp2 = getXY(car_s + 90, 2 + (4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

                            pts_x.push_back(next_wp0[0]);
                            pts_y.push_back(next_wp0[1]);

                            pts_x.push_back(next_wp1[0]);
                            pts_y.push_back(next_wp1[1]);

                            pts_x.push_back(next_wp2[0]);
                            pts_y.push_back(next_wp2[1]);

                            /* Making coordinates to local car coordinates */
                            for (int i = 0; i < pts_x.size(); i++)
                            {
                                double shift_x = pts_x[i] - ref_x;
                                double shift_y = pts_y[i] - ref_y;

                                pts_x[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
                                pts_y[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
                            }

                            /* Create the spline. */
                            tk::spline s;
                            s.set_points(pts_x, pts_y);

                            //For the smooth transition, we are adding previous path points
                            for (int i = 0; i < prev_size; i++)
                            {
                                next_x_vals.push_back(previous_path_x[i]);
                                next_y_vals.push_back(previous_path_y[i]);
                            }

                            /* Calculate distance y position on 30 m ahead. */
                            double target_y = s(target_x);
                            double target_dist = sqrt(target_x * target_x + target_y * target_y);
                            double x_add_on = 0;
                            double N = target_dist / (time_per_frame * ref_vel / 2.24);

                            for (int i = 1; i < 50 - prev_size; i++)
                            {
                                /*
                                * Formula for calculating number of segments ´N´, N * 0.02* velocity = 30.0
                                */
                                double x_point = x_add_on + target_x / N;
                                double y_point = s(x_point);

                                x_add_on = x_point;

                                double x_ref = x_point;
                                double y_ref = y_point;

                                /* Rotate back to world coordinates from the vehicle coordinate */
                                x_point = ref_x + x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
                                y_point = ref_y + x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

                                next_x_vals.push_back(x_point);
                                next_y_vals.push_back(y_point);
                            }


                            msgJson["next_x"] = next_x_vals;
                            msgJson["next_y"] = next_y_vals;

                            auto msg = "42[\"control\"," + msgJson.dump() + "]";

                            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                        }  // end "telemetry" if
                    }
                    else
                    {
                        // Manual driving
                        std::string msg = "42[\"manual\",{}]";
                        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                    }
                }  // end websocket if
        }); // end h.onMessage

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) 
        {
        std::cout << "Connected!!!" << std::endl;
        });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
        char *message, size_t length) 
        {
            ws.close();
            std::cout << "Disconnected" << std::endl;
        });

    int port = 4567;
    if (h.listen("127.0.0.1", port)) 
    {
        std::cout << "Listening to port " << port << std::endl;
    }
    else 
    {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }

    h.run();
}