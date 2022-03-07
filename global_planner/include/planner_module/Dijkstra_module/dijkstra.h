
#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include <iostream>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <set>

#include "../../global_planner/path_data.h"
#include "../utils/array_hasher.h"


#include "../A_star_module/astar.h"

using std::array;
using std::cout;
using std::endl;
using std::string;
using std::vector;


class DijkstracClass : public pathPoint 
{
private:

    int motion_model_[8][3];
    float goal_angle_;
    array<int, 2> start_xy_;
    array<int, 2> goal_xy_;
    
    array<int, 2> curr_xy_;

    int map_width_;
    int map_height_;

    struct nodeInfo 
    {
        array<int, 2>  parent;
        int node_state = 0;
        int node_gcost;
        // int node_fcost;
        float heading;
    };

    // ArrayHasher hasher;

    std::unordered_map<array<int, 2>, nodeInfo, ArrayHasher> all_nodes;


    std::unordered_set< array<int, 2>, ArrayHasher > open_nodes;
    // std::set< array<int, 2> > open_nodes;

    vector<int8_t> grid_map;

    vector<int8_t> grid_map_subsample;
    static const int subsample_size = 2;

    float resize_img_ratio = 2.0;

    int search_time_out_ms_;

    std::deque< pathPoint > path_;


    void explore_one_node(array<int, 2> curr_xy);
    void explore_one_ite(vector< array<int, 2>> active_nodes);
    vector< array<int, 2>> find_min_cost_nodes();

    void build_motion_model();
    void build_parent_grid();
    void print_init_info();
    void draw_start_goal_on_map();
    int twoD_to_oneD(const int x, const int y, const int width, const int height);

    float rectify( float a);

public:
    DijkstracClass();

    ~DijkstracClass();

    void setup(const int startnode[], const int goalnode[], const float goal_angle, const vector<int8_t>& map, const int height, const int width, const int timeout_ms  );

    bool FLAG_update_map_for_view = true;

    
    bool search();
    std::deque< pathPoint > get_path();
};


#endif