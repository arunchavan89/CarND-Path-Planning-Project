#include "path_planner.h"
#include "spline.h"
//#include "helpers.h"

void PathPlanner::path_planner_init(int lane_number, double ref_vel)
{
    lane = lane_number;                                   // Middle lane                            
    ref_vel_m_per_sec = ref_vel / 2.24;      // velocity in m/sec
    target_x = 30.0;                            // creating spline till next 30 meters
    time_per_frame = 0.02;                      // time in second per frame; 50 FPS
}

void PathPlanner::path_planner(s_car_parameters_t car_params,
    std::vector<double>previous_path_x, std::vector<double>previous_path_y,
    std::vector<double> map_waypoints_s, std::vector<double>map_waypoints_x, std::vector<double>map_waypoints_y,
    std::vector<double> &next_x_vals, std::vector<double> &next_y_vals)
{

    //Example code:
#if 0
    double path_x = 0.0;
    double path_y = 0.0;
    double dist_inc = 0.5; // The car moves 50 times in 1 second. move 0.5 meter 50 times = 25 m/s = 50MPH
    for (int i = 0; i < 50; i++)
    {
        path_x = car_x + (dist_inc * i) * cos(deg2rad(car_yaw));
        path_y = car_y + (dist_inc * i) * sin(deg2rad(car_yaw));
        next_x_vals.push_back(path_x);
        next_y_vals.push_back(path_y);
    }
#endif

    // Size of the previous path
    prev_size = previous_path_x.size();

    // If previous states are almost empty, use the car as a starting point
    if (prev_size < 2)
    {
        ref_x = car_params.car_x;
        ref_y = car_params.car_y;
        ref_yaw = deg2rad(car_params.car_yaw);

        //Use two points thats makes path tangent to the car
        double prev_car_x = ref_x - cos(car_params.car_yaw);
        double prev_car_y = ref_y - sin(car_params.car_yaw);

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
    std::vector<double> next_wp0 = getXY(car_params.car_s + 30, 2 + (4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    std::vector<double> next_wp1 = getXY(car_params.car_s + 60, 2 + (4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    std::vector<double> next_wp2 = getXY(car_params.car_s + 90, 2 + (4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

    pts_x.push_back(next_wp0[0]);
    pts_y.push_back(next_wp0[1]);

    pts_x.push_back(next_wp1[0]);
    pts_y.push_back(next_wp1[1]);

    pts_x.push_back(next_wp2[0]);
    pts_y.push_back(next_wp2[1]);

    // Making coordinates to local car coordinates.
    for (int i = 0; i < pts_x.size(); i++)
    {
        double shift_x = pts_x[i] - ref_x;
        double shift_y = pts_y[i] - ref_y;

        pts_x[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
        pts_y[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
    }

    create_spline_trajectory(previous_path_x, previous_path_y, next_x_vals, next_y_vals);

}

void PathPlanner::create_spline_trajectory(std::vector<double>previous_path_x, std::vector<double>previous_path_y,
    std::vector<double> &next_x_vals, std::vector<double> &next_y_vals)
{
    // Create the spline.
    tk::spline s;
    s.set_points(pts_x, pts_y);

    //For the smooth transition, we are adding previous path points
    for (int i = 0; i < prev_size; i++)
    {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    // Calculate distance y position on 30 m ahead.
    double target_y = s(target_x);
    double target_dist = sqrt(target_x * target_x + target_y * target_y);

    double x_add_on = 0;

    for (int i = 1; i < 50 - prev_size; i++)
    {
        /*
        * Formula for calculating number of segments ´N´, N * 0.02* velocity = 30.0
        */

        double N = target_dist / (time_per_frame * ref_vel_m_per_sec);
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
}