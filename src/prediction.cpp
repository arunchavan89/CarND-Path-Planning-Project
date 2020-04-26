#include "prediction.h"

void Prediction::prediction_init()
{
    /*Maximum speed allowed*/
    max_speed_MPH = 49.0;
    max_speed_MPH_1 = 47.0;    
    offset = 0.224;
    offset_pos_accln = 0.224;
    emergency_braking = 15.0;
    speed_car_ahead = 0.0;    
}

void Prediction::prediction_doit(int prev_size, double end_path_s, std::vector<std::vector<double>>sensor_fusion,
    int &lane, double &ref_vel)
{
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

        if ((d < (2 + 4 * lane + 2)) && (d > (2 + 4 * lane - 2)))
        {
            if ((check_car_s > car_s) && (check_car_s - car_s) < 30)
            {
                car_ahead = true;
                speed_car_ahead = check_speed;
                if ((check_car_s - car_s) < 5)
                {
                    emergency_brake = true;
                }
            }
        }

        if ((d > 0) && (d < 4))
        {
            if (((check_car_s > car_s) && (check_car_s - car_s) < 35) ||
                ((car_s > check_car_s) && (car_s - check_car_s) < 10))
            {
                car_lane_0 = true;
            }
        }
        if ((d > 4) && (d < 8))
        {
            if (((check_car_s > car_s) && (check_car_s - car_s) < 35) ||
                ((car_s > check_car_s) && (car_s - check_car_s) < 10))
            {
                car_lane_1 = true;
            }
        }
        if ((d > 8) && (d < 12))
        {
            if (((check_car_s > car_s) && (check_car_s - car_s) < 35) ||
                ((car_s > check_car_s) && (car_s - check_car_s) < 10))
            {
                car_lane_2 = true;
            }
        }

    }

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
            //if (emergency_brake)
            {
                //ref_vel -= emergency_braking;
            }
            // else
            {
                if (ref_vel != speed_car_ahead)
                {
                    ref_vel -= offset;
                }
            }
        }
    }
    else if (ref_vel < max_speed_MPH)
    {
        if (ref_vel < max_speed_MPH_1)
        {
            ref_vel += offset_pos_accln;
        }
        if (ref_vel < max_speed_MPH)
        {
            ref_vel += offset;
        }
    }
}
