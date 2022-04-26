#include "Grid_map.h"
#include <random>

Grid_map::Grid_map(){}
Grid_map::Grid_map(const Grid_map& object){
    x_range = object.x_range;
    y_range = object.y_range;
    grid_map = object.grid_map;
}
Grid_map::Grid_map(int x_range, int y_range) : x_range(x_range), y_range(y_range){
//    x_range = 400;
//    y_range = 400;

    // grid_map[x][y] = 0 or 1 (0: free, 1: obstacle)
    grid_map = new int*[x_range];
    for (int i = 0; i < x_range; i++){
        grid_map[i] = new int[y_range];
    }
}
void Grid_map::generate_random_obstacle(int num_obstacle) {
    int obstacle_size = x_range/10;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dis_x(30,320);
    std::uniform_int_distribution<int> dis_y(30,320);

    int obs = 0;
    while (obs < num_obstacle){
        int rand_x = dis_x(gen);
        int rand_y = dis_y(gen);

        obstacle_mid_point_set.push_back(std::make_pair(rand_x, rand_y));

        for (int i = rand_x; i < rand_x + obstacle_size; i++){
            for (int j = rand_y; j < rand_y + obstacle_size; j++){
                if (i < x_range && j < y_range && grid_map[i][j] == 0){
                    grid_map[i][j] = 1;
                    obstacle_set.push_back(std::make_pair(i, j));
                }
            }
        }
        obs++;
    }

//    // generate initial obstacle block
//    for (int i = 0; i < x_range * 2/4; i++){
//        for (int j = y_range * 1/3; j < y_range * 1/3 + y_range/50; j++){
//            if (grid_map[i][j] == 0){
//                grid_map[i][j] = 1;
//                obstacle_set.push_back(std::make_pair(i, j));
//            }
//        }
//    }
//
//    for (int i = x_range * 2/4; i < x_range; i++){
//        for (int j = y_range * 2/3; j < y_range * 2/3 + y_range/50; j++){
//            if (grid_map[i][j] == 0){
//                grid_map[i][j] = 1;
//                obstacle_set.push_back(std::make_pair(i, j));
//            }
//        }
//    }
}
int** Grid_map::get_grid_map(){
    return grid_map;
}
std::vector<std::pair<int,int>> Grid_map::get_obstacle_set(){
    return obstacle_set;
}
std::vector<std::pair<int,int>> Grid_map::get_obstacle_mid_point_set(){
    return obstacle_mid_point_set;
}
Grid_map::~Grid_map(){
    for (int i = 0; i < x_range; i++){
        delete[] grid_map[i];
    }
    delete[] grid_map;
}