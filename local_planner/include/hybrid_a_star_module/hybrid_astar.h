
#ifndef HYBRID_ASTAR_H
#define HYBRID_ASTAR_H

#include <iostream>
#include <queue>
#include <unordered_map>
#include <map>
#include <math.h>
// #include <opencv2/core.hpp>
#include <algorithm>
#include <set>
#include <chrono>

#include "utils/array_hasher.cpp"

#include "../local_planner/path_data.h"

using std::array;
using std::cout;
using std::endl;
using std::string;
using std::vector;


class Hybrid_astar_class : public pathPoint
{
private:

    int time_out_ms_;

    vector< vector<float>> motion_model_;

    array<int, 3> start_grid_;
    array<float, 3> start_pose_;

    array<int, 3> goal_grid_;
    array<float, 3> goal_pose_;

    array<float, 3> close_goal_pose_;

    float start_angle_, goal_angle_, close_goal_angle_; 

    // struct gridFinePose 
    // {
        
    // };
    

    struct gridInfo
    {
        float gcost = 0;
        float fcost = 0;
        int state = 0;       // 0:new   1:open   2:closed
        int steer_type = 0;  // 0-5, total 6 types
        // float fine_pose_x, fine_pose_y, fine_pose_yaw;
        array<float, 3> fine_pose;
        
        array<int,3> parent;
    };
    
    std::unordered_map<array<int, 3>, gridInfo , ArrayHasher3> all_grids_;

    // std::unordered_map<array<int, 3>, array<int,   3>, ArrayHasher3> grid_parent;
    // std::unordered_map<array<int, 3>, array<float, 3>, ArrayHasher3> grid_fine_pose;
    // std::unordered_map<array<int, 3>, array<int,   4>, ArrayHasher3> grid_status;
    /*
    0 : gcost.  
    1 : fcost.
    2 : state :  0:new   1:open   2:closed
    3 : steer_type:  0-5 
    */

    struct goalTolerance
    {
       float max_distance_error;
       float max_heading_error;
    };

    goalTolerance goal_tolerance;
   


    int fine_map_width_, fine_map_height_, grid_map_width_, grid_map_height_;

    vector<int8_t> map_grid;

    

    // cv::Mat map_grid;     // low resolution map
    // cv::Mat map_fine;     // fine resolution map
    // cv::Mat map_for_view; // the copy used for visulization, constantly being modiified
    // cv::Mat map_collision; // the copy used for check collision

    const float fine_ratio_ = 0.1;
    const float angle_resolution_ = M_PI/18.0; // radius
    // const int obstacle_scan_range_ = 15;
    // const int map_inflation_size_ = 5;

    const string window_name_ = "map";

    float step_length_ ;
    float turning_raius_  ;
    float turning_angle_ ;

    bool FLAG_reach_goal_ = false;


    std::deque< pathPoint > path_;

    void build_motion_model();
    // void map_inflation(cv::Mat* mapIn, cv::Mat* mapOut, int infla_size);
    void print_init_info();

    // void draw_start_goal_on_map();
    // void draw_arc(array<float, 3> node_pose, array<float, 3> next_pose, int radius, float arc_angle, int motion_type , int thickness, cv::Scalar color);
    
    // int check_collision( float x, float y, int scan_range );
    int improve_hcost( array<float, 3> node_pose, int hcost, int scan_range, bool same_steer);
    void check_if_reach_goal( array<float, 3> node_pose, array<float, 3> in_goal_pose, array<int, 3> in_curr_grid);

    int twoD_to_oneD(const int x, const int y, const int width, const int height);


public:
    Hybrid_astar_class();
    void setup(const int timeout_ms ,int startnode[], float s_angle, array<float,3> goalpose , const float map_width_meter, const float map_height_meter, const int map_width_grid, const int map_height_grid, const std::vector<int8_t>& map);// cv::Mat map);

    ~Hybrid_astar_class();

    bool FLAG_update_map_for_view = true;

    void explore_one_node(array<float, 3> curr_pose, array<int, 3> curr_grid);
    void explore_one_ite(vector< array<int, 3>> active_nodes);
    vector< array<int,3>> find_min_cost_nodes();
    bool search();
    std::deque< pathPoint >  get_path();
    void print_path();


};


#endif