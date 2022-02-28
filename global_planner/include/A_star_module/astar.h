
#ifndef ASTAR_H
#define ASTAR_H

#include <iostream>
#include <queue>
#include <unordered_map>
#include <map>
#include <opencv2/core.hpp>
#include <algorithm>
#include <set>

#include "../global_planner/path_data.h"

#include "utils/array_hasher.cpp"



class AstarClass : public pathPoint
{
private:

    int motion_model_[8][4];

    std::array<int, 2> start_xy;
    std::array<int, 2> goal_xy;

    float goal_angle_;
    
    int map_width;
    int map_height;


    struct nodeInfo 
    {
        std::array<int, 2>  parent;
        int node_state = 0;
        int node_gcost;
        int node_fcost;
        float heading;
    };
    
    std::unordered_map<std::array<int, 2>, nodeInfo, ArrayHasher> all_nodes;


    // cv::Mat mapCv;
    // cv::Mat map_for_view;
    std::vector<int8_t> grid_map;

    std::vector<int8_t> grid_map_subsample;
    static const int subsample_size = 2;

    float resize_img_ratio = 2.0;

    // struct pathPoint
    // {
    //     std::array<int, 2> xy;
    //     float heading;
    // };
    
    std::deque< pathPoint > path;

    void explore_one_node(std::array<int, 2> curr_xy);
    void explore_one_ite(std::vector< std::array<int, 2>> active_nodes);
    std::vector< std::array<int, 2>> find_min_Fcost_nodes();
    void find_min_FHcost_nodes( std::array<int, 2>& best_approx_goal_xy );

    void build_motion_model();
    void build_parent_grid();
    void print_init_info();
    // void draw_start_goal_on_map();

    int twoD_to_oneD(const int x, const int y, const int width, const int height);

    int compute_h_cost_Euclidean(const std::array<int,2> n );
    int compute_h_cost_Manhattan(const std::array<int,2> n );
    int compute_h_cost_Chebyshev(const std::array<int,2> n );

    float rectify( float a);

public:

    AstarClass();
    void setup(const int startnode[], const int goalnode[], const float goal_angle, const std::vector<int8_t>& map, const int height, const int width );

    AstarClass(const int startnode[], const int goalnode[], const std::vector<int8_t>& map, const int height, const int width );

    ~AstarClass();

    bool FLAG_update_map_for_view = true;

    


    
    void search();
    std::deque< pathPoint > get_path();
    void print_path();


};


#endif