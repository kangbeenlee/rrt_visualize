#pragma once
#include <iostream>
#include <vector>



class Grid_map{
public:
    int** grid_map;
    std::vector<std::pair<int,int>> obstacle_set;
    std::vector<std::pair<int,int>> obstacle_mid_point_set;

    int x_range;
    int y_range;
    Grid_map();
    Grid_map(const Grid_map& object);
    Grid_map(int x_range, int y_range);
    void generate_random_obstacle(int num_obstacle);
    int** get_grid_map();
    std::vector<std::pair<int,int>> get_obstacle_set();
    std::vector<std::pair<int,int>> get_obstacle_mid_point_set();
    ~Grid_map();
};


