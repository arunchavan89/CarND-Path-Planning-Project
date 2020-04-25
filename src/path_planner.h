# ifndef _PATH_PLANNER_H_
# define _PATH_PLANNER_H_


#include <vector>

class PathPlanner
{
public:

    struct s_car_parameters_t {
        double car_x;
        double car_y;
        double car_s;
        double car_d;
        double car_yaw;
        double car_speed;
    }car_parameters_t;

    void path_planner_init(double ref_vel);

    void path_planner(s_car_parameters_t car_params,
        std::vector<double>previous_path_x, std::vector<double>previous_path_y,
        std::vector<double> map_waypoints_s, std::vector<double>map_waypoints_x, std::vector<double>map_waypoints_y,
        std::vector<double> &next_x_vals, std::vector<double> &next_y_vals);

    void create_spline_trajectory(std::vector<double>previous_path_x, std::vector<double>previous_path_y,
        std::vector<double> &next_x_vals, std::vector<double> &next_y_vals);

    std::vector<double> pts_x;
    std::vector<double> pts_y;

    // Reference x, y, and yaw states
    double ref_x;
    double ref_y;
    double ref_yaw;

    // Size of the previous path
    int prev_size;

    double ref_vel; //MPH

    /*--------------------------------------Helper Functions------------------------------------------------*/

    constexpr double pi() { return 3.141592; }
    double deg2rad(double x) { return x * pi() / 180; }
    double rad2deg(double x) { return x * 180 / pi(); }

    // Transform from Frenet s,d coordinates to Cartesian x,y
    std::vector<double> getXY(double s, double d, const std::vector<double> &maps_s,
        const std::vector<double> &maps_x,
        const std::vector<double> &maps_y)
    {
        int prev_wp = -1;

        while (s > maps_s[prev_wp + 1] && (prev_wp < (int)(maps_s.size() - 1)))
        {
            ++prev_wp;
        }

        int wp2 = (prev_wp + 1) % maps_x.size();

        double heading = atan2((maps_y[wp2] - maps_y[prev_wp]),
            (maps_x[wp2] - maps_x[prev_wp]));
        // the x,y,s along the segment
        double seg_s = (s - maps_s[prev_wp]);

        double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
        double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

        double perp_heading = heading - pi() / 2;

        double x = seg_x + d * cos(perp_heading);
        double y = seg_y + d * sin(perp_heading);

        return { x,y };
    }

private:

    int lane;   

    double ref_vel_m_per_sec;

    double target_x;

    double time_per_frame;
};
# endif /*_PATH_PLANNER_H_*/
