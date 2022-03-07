
#include <iostream>
#include <string>
#include <stdexcept>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <limits>
#include <math.h>
#include <chrono>

#include "astar.h"

using std::chrono::high_resolution_clock;
using std::vector;
using std::array;


void AstarClass::setup( const int startnode[], const int goalnode[], const float goal_angle, const std::vector<int8_t>& map, const int height, const int width, const int timeout_ms )
{
    start_xy[0] = startnode[0];
    start_xy[1] = startnode[1];
    goal_xy[0] = goalnode[0];
    goal_xy[1] = goalnode[1];

    goal_angle_ = goal_angle;

    search_time_out_ms_ = timeout_ms;

    all_nodes.clear();
    path.clear();
    open_nodes.clear();

    all_nodes[start_xy].parent = start_xy;
    all_nodes[start_xy].node_gcost = 0;
    all_nodes[start_xy].node_state = 1;
    open_nodes.insert(start_xy);

    grid_map = map;

    // int h_count = 0;
    // int w_count = 0;
    // while ( h_count < height )
    // {
    //     w_count = 0;
    //     while (w_count < width)
    //     {
    //         grid_map_subsample.push_back( map[ h_count*width + w_count ] );
    //         w_count += subsample_size;
    //     }
    //     h_count += subsample_size;
    // }

    map_width = width;   
    map_height = height; 
    // map_width = w_count;   
    // map_height = h_count; 

    std::cout << "Setup A* ";// << std::endl;
    std::cout << "map width: " << map_width << " ;height: "  << map_height  ;
    std::cout << "  start_xy: " << start_xy[0] << ", "  << start_xy[1] ;
    std::cout << "  goal_xy: " << goal_xy[0] << ", "  << goal_xy[1]  << std::endl;



    // std::set<int> alv;
    // int ct_1= 0;
    // int ct_0= 0;
    // int ct_100= 0;

    // for (auto p : map)
    // {
    //     alv.insert( int( p ) );
    //     if(p==0  ) { ct_0  ++;}
    //     if(p==-1 ) { ct_1  ++;}
    //     if(p==100) { ct_100++;}
    // }

    // std::cout << ct_1 << std::endl;
    // std::cout << ct_0 << std::endl;
    // std::cout << ct_100 << std::endl;

    // for( auto p : alv)
    // {
    //     std::cout << p << std::endl;
    // }


}

AstarClass::AstarClass(): pathPoint()
{
    build_motion_model();
}

AstarClass::~AstarClass()
{
}



void AstarClass::build_motion_model(){
    int xs[8] = {-1,0,1,1,1,0,-1,-1};  // x movement
    int ys[8] = {1,1,1,0,-1,-1,-1,0};  // y movement
    int cost[8] = { 14,10,14,10,14,10,14,10 }; // cost
    int headings[8] = { 135, 90, 45, 0, 315, 270, 225, 180 }; // heading angles
    for(int i = 0; i<8; i++){
        motion_model_[i][0] = xs[i];
        motion_model_[i][1] = ys[i];
        motion_model_[i][2] = cost[i];
        motion_model_[i][3] = headings[i];
    }
}

int AstarClass::twoD_to_oneD(const int x, const int y, const int width, const int height)
{
    int out= y*width + x;
    return out;
}



void AstarClass::explore_one_node(const array<int, 2> curr_xy){
    // move it into closed 
    all_nodes[curr_xy].node_state = 2;

    open_nodes.erase(curr_xy);

    for(auto mm : motion_model_){
        int n_x = curr_xy[0] + mm[0];
        int n_y = curr_xy[1] + mm[1];
        int edge_cost = mm[2];
        float heading = float( mm[3] );

        if(0<= n_x && n_x < map_width && 0<= n_y && n_y < map_height){

            int grid_index = twoD_to_oneD(n_x, n_y, map_width, map_height);
            int obs_prob = int(grid_map[ grid_index ]);
            // int obs_prob = int(grid_map_subsample[ grid_index ]);
            if( obs_prob != 100 ){
                // if( obs_prob != -1 ){
                //     std::cout << n_x << " " << n_y << " " << obs_prob << std::endl;
                // }

                array<int,2> n_node = {n_x, n_y};
                // if(node_parent.count(n_node) == 0){
                if(all_nodes[n_node].node_state == 0){
                    all_nodes[n_node].parent = curr_xy;
                    all_nodes[n_node].node_state = 1;
                    all_nodes[n_node].node_gcost = all_nodes[curr_xy].node_gcost + edge_cost;

                    int hcost = compute_h_cost_Euclidean(n_node);

                    all_nodes[n_node].node_fcost = all_nodes[n_node].node_gcost + hcost;
                    all_nodes[n_node].heading = heading;
                    open_nodes.insert(n_node);
                } 
                if(all_nodes[n_node].node_state == 1 && all_nodes[n_node].parent != curr_xy)
                {
                    int old_f_cost = all_nodes[n_node].node_fcost;
                    int hcost = compute_h_cost_Euclidean(n_node);
                    int new_g_cost = all_nodes[curr_xy].node_gcost + edge_cost;
                    int new_f_cost = new_g_cost + hcost;
                    if(new_f_cost < old_f_cost)
                    {
                        all_nodes[n_node].parent = curr_xy;
                        all_nodes[n_node].node_gcost = new_g_cost;
                        all_nodes[n_node].node_fcost = new_f_cost;

                    }
                }
            }
        }
    }
}



vector< array<int, 2>> AstarClass::find_min_Fcost_nodes(){
    vector< array<int, 2>> out;
    int min_cost = std::numeric_limits<int>::max(); 

    // if a node exist in node_parent
    // for(auto n : all_nodes){
    //     array<int, 2> node = n.first;
    for(auto n : open_nodes){
        // array<int, 2> node = n.first;
        array<int, 2> node = n;
        // if(all_nodes[node].node_state == 1){
            int cost = all_nodes[node].node_fcost;
            if(cost < min_cost){
                min_cost = cost;
                out.clear();
                out.push_back(node);
            }
            if(cost == min_cost){
                out.push_back(node);
            }
        // }
    }
    return out;
}

void AstarClass::find_min_FHcost_nodes( array<int, 2>& best_approx_goal_xy )
{
    int min_Fcost = std::numeric_limits<int>::max(); 
    int min_Hcost = std::numeric_limits<int>::max(); 

    vector< array<int, 2>> min_F_nodes;

    for(auto n : all_nodes){
        array<int, 2> node = n.first;
        if( all_nodes[node].node_state != 0 ){
            int cost = all_nodes[node].node_fcost;
            if(cost < min_Fcost){
                min_Fcost = cost;
                min_F_nodes.clear();
                min_F_nodes.push_back(node);
            }
            else if(cost == min_Fcost){
                min_F_nodes.push_back(node);
            }
        }
    }

    for(auto n : min_F_nodes){
        int cost = all_nodes[n].node_fcost - all_nodes[n].node_gcost;
        if(cost < min_Hcost){
            min_Hcost = cost;
            best_approx_goal_xy = n;
        }
    }
}



void AstarClass::explore_one_ite(vector< array<int, 2>> active_nodes){
    for(vector<array<int, 2>>::iterator it = active_nodes.begin(); it != active_nodes.end(); ++it){
        explore_one_node(*it);
    }
}


bool AstarClass::search(){

    std::cout << "A* looking for path. ";// << std::endl;

    high_resolution_clock::time_point time1 = high_resolution_clock::now();
    high_resolution_clock::time_point time_temp;
    int count = 0;

    while(all_nodes[goal_xy].node_state == 0){

        // check time used
        time_temp = high_resolution_clock::now();
        auto duration_temp = std::chrono::duration_cast<std::chrono::microseconds>(time_temp-time1);
        int time_past = int(duration_temp.count()/1000.0);
        if(time_past >= search_time_out_ms_)
        {
            std::cout << "Search Timeout" << std::endl;
            return false;
        }

        // check if all open nodes are already explored, and still not find goal
        auto opennodes = find_min_Fcost_nodes();
        if(opennodes.size() == 0)
        {
            std::cout << "No valid path found. \nGetting the path closest to the goal." << std::endl;
            find_min_FHcost_nodes(goal_xy);
            return true;
        }
        
        // regular A* exploration
        explore_one_ite( opennodes );

        count ++;
    }

    high_resolution_clock::time_point time2 = high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(time2-time1);

    std::cout << "Found the goal. Time used " << int(duration.count()/1000.0) << " ms." << std::endl;
    return true;
}



std::deque< AstarClass::pathPoint > AstarClass::get_path(){
    array<int, 2> get_path_curr_node = goal_xy;
    
    while (path.front().xy != start_xy)
    {
        pathPoint pp;
        pp.xy = get_path_curr_node;
        // pp.xy[0] *= subsample_size;
        // pp.xy[1] *= subsample_size;

        // pp.heading = all_nodes[get_path_curr_node].heading;

        get_path_curr_node = all_nodes[get_path_curr_node].parent;
        
        path.push_front( pp );
    }
    for(int i = 0; i<path.size(); i++)
    {
        float dx = path[i+1].xy[0] - path[i].xy[0];
        float dy = path[i+1].xy[1] - path[i].xy[1];
        path[i].heading = rectify( atan2(dy,dx) ) * 180.0 / M_PI;
    }
    path[ path.size()-1 ].heading = goal_angle_* 180.0 / M_PI;
    return path;

}



void AstarClass::print_init_info(){
    std::cout << "\nAstarClass start" 
    << "\n\nmap size:  \nw: " << map_width << " h: " << map_height
    << "\n\nStart location: \n" << start_xy[0] << " " << start_xy[1]
    << "\n\nGoal location : \n" << goal_xy[0] << " " << goal_xy[1]
    << std::endl;
}



float AstarClass::rectify( float a)
{
    float angle = a;
    while (angle > 2*M_PI)
    {
        angle -= 2*M_PI;
    }
    while (angle < 0)
    {
        angle += 2*M_PI;
    }
    return angle;
}





int AstarClass::compute_h_cost_Euclidean( const array<int,2> n ){
    float h = 0;

    int dx = n[0] - goal_xy[0] ;
    int dy = n[1] - goal_xy[1] ;

    h = sqrt( dx*dx + dy*dy ) *10;

    return int(h) ;
}

int AstarClass::compute_h_cost_Manhattan( const std::array<int,2> n ){
    int h = 0;

    int dx = abs( n[0] - goal_xy[0] );
    int dy = abs( n[1] - goal_xy[1] );

    h = ( dx + dy ) *10;

    return int(h) ;
}

int AstarClass::compute_h_cost_Chebyshev( const std::array<int,2> n ){
    int h = 0;

    int dx = abs( n[0] - goal_xy[0] );
    int dy = abs( n[1] - goal_xy[1] );

    h = MAX(dx, dy) *10;

    return int(h) ;
}


