#include "map.h"

void Map::read_map(std::string map_file)
{
    std::ifstream in_map_(map_file.c_str(), std::ifstream::in);

    // Load up map values for waypoint's x,y,s and d normalized normal vectors  
    std::string line;
    while (getline(in_map_, line)) 
    {
        std::istringstream iss(line);       
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
}