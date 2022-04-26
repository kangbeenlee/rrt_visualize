#pragma once

#include "Grid_map.h"
#include <iostream>
#include <cmath>
#include <random>
#include <list>
#include <set>



class Node{
public:
    int id;
    double x;
    double y;
    Node* parent;

    Node(double x, double y) : x(x), y(y) {}

    double get_distance_to(const Node* near_node){
        return sqrt(pow(x - near_node->x, 2) + pow(y - near_node->y, 2));
    }
    double get_distance_to(const int& to_x, const int& to_y)const{
        return sqrt(pow(x - to_x, 2) + pow(y - to_y, 2));
    }
    // for vector calculation
    Node operator+(const Node& another)const{
        return Node(x + another.x, y + another.y);
    }
    Node operator-(const Node& another)const{
        return Node(x - another.x, y - another.y);
    }
    Node operator*(const double multiplier)const{
        return Node(x * multiplier, y * multiplier);
    }
};

class Graph{
    std::list<Node*> node_list;
public:
    void add_node(Node* new_node){
        node_list.push_back(new_node);
    }
    std::list<Node*> get_node_list(){
        return node_list;
    }
    ~Graph(){
        for (Node* node_ptr : node_list){
            delete node_ptr;
        }
    }
};

class RRT {
public:
    // parameter
    int num_sampling;
    int expand_length;
    int target_radius;

    Grid_map* grid_map;
    Graph graph;

    // root & target node
    int start_x, start_y, target_x, target_y;
    bool success;

    RRT(int start_x, int start_y, int target_x, int target_y, int x_range, int y_range)
            : start_x(start_x), start_y(start_y), target_x(target_x), target_y(target_y){
        // change parameter
        num_sampling = 400;
        expand_length = 30;
        target_radius = 30;

        grid_map = new Grid_map(x_range, y_range);
        grid_map->generate_random_obstacle(10);
        success = false;
    }
    ~RRT(){
        delete grid_map;
    }

    bool collision_free(Node* from, Node* to){
        double x1 = from->x;
        double y1 = from->y;
        double x2 = to->x;
        double y2 = to->y;

        std::vector<std::pair<int,int>> point_set;

        if ( std::abs(x1-x2) > std::abs(y1-y2)){
            int gap = std::abs(x1-x2);
            for (int i = 0; i < gap + 1; i++){
                int x = x1 + (x2 - x1) / (gap + 1e-6) * i;
                int y = round((y2 - y1) / (x2 - x1 + 1e-6) * (x - x1) + y1);
                point_set.push_back(std::make_pair(x, y));
            }
        } else {
            int gap = std::abs(y1-y2);
            for (int i = 0; i < gap + 1; i++){
                int y = y1 + (y2 - y1) / (gap + 1e-6) * i;
                int x = round((y - y1) * (x2 - x1) / (y2 - y1 + 1e-6) + x1);
                point_set.push_back(std::make_pair(x, y));
            }
        }

        // collision detection
        for (auto itr = point_set.begin(); itr != point_set.end(); itr++){
            if (grid_map->get_grid_map()[(*itr).first][(*itr).second] == 1) {
                return false;
            }
        }
        return true;
    }

    // new node generation
    Node* steer(Node* nearest_node, const Node& rand_node, double nearest_distance){
        Node* new_node;
        if (nearest_distance > expand_length) {
            Node nearest_p(nearest_node->x, nearest_node->y);
            Node rand_p(rand_node.x, rand_node.y);
            Node diff_p = rand_p - nearest_p; // difference b.w. nearest node and random node
            Node normed_diff_p(diff_p.x / (sqrt(pow(diff_p.x, 2) + pow(diff_p.y, 2)) + 1e-6),
                               diff_p.y / (sqrt(pow(diff_p.x, 2) + pow(diff_p.y, 2)) + 1e-6));
            Node rough_new_node = (normed_diff_p * expand_length) + nearest_p;
            new_node = new Node(round(rough_new_node.x), round(rough_new_node.y));
        } else {
            new_node = new Node(rand_node.x, rand_node.y);
        }
        return new_node;
    }
};