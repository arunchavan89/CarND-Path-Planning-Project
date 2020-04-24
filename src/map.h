# ifndef _MAP_H_
# define _MAP_H_

#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <sstream>

class Map
{
public:

    void read_map(std::string map_file);

    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    std::vector<double> map_waypoints_x;
    std::vector<double> map_waypoints_y;
    std::vector<double> map_waypoints_s;
    std::vector<double> map_waypoints_dx;
    std::vector<double> map_waypoints_dy;
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
};
# endif /*_MAP_H_*/
