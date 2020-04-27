# ifndef _PREDICTION_H_
#include <vector>

class Prediction
{
public:
    void prediction_init();

    void prediction_doit(int prev_size, double end_path_s, std::vector<std::vector<double>>sensor_fusion,
        int &lane, double &ref_vel);
    /*Maximum speed allowed*/
    double max_speed_MPH;
    double max_speed_MPH_1;
    double car_s;
    double offset;
    double offset_pos_accln;
    double emergency_braking;
    double speed_car_ahead;    
   
private:
    

};
# endif /*_PREDICTION_H_*/
