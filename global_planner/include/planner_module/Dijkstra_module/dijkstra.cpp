
#include <iostream>
#include <string>
#include <stdexcept>
#include <limits>
#include <chrono>
#include <math.h>

#include "dijkstra.h"


using std::chrono::high_resolution_clock;

DijkstracClass::DijkstracClass() : pathPoint()
{
    build_motion_model();
}

DijkstracClass::~DijkstracClass()
{
}

void DijkstracClass::setup( const int startnode[], const int goalnode[], const float goal_angle, const vector<int8_t>& map, const int height, const int width, const int timeout_ms )
{
    start_xy_[0] = startnode[0];
    start_xy_[1] = startnode[1];
    goal_xy_[0] = goalnode[0];
    goal_xy_[1] = goalnode[1];

    goal_angle_ = goal_angle;

    search_time_out_ms_ = timeout_ms;

    all_nodes.clear();
    path_.clear();
    open_nodes.clear();

    all_nodes[start_xy_].parent = start_xy_;
    all_nodes[start_xy_].node_gcost = 0;
    all_nodes[start_xy_].node_state = 1;
    open_nodes.insert(start_xy_);
    // open_nodes.

    grid_map = map;

    map_width_ = width;   
    map_height_ = height; 

    // std::cout << "Setup Dijkstra ";// << std::endl;
    // std::cout << "map width: " << map_width_ << " ;height: "  << map_height_  ;
    // std::cout << "  start_xy: " << start_xy_[0] << ", "  << start_xy_[1] ;
    // std::cout << "  goal_xy: " << goal_xy_[0] << ", "  << goal_xy_[1]  << std::endl;

}

void DijkstracClass::build_motion_model(){
    int xs[8] = {-1,0,1,1,1,0,-1,-1};
    int ys[8] = {1,1,1,0,-1,-1,-1,0};
    int cost[8] = { 14,10,14,10,14,10,14,10 };
    for(int i = 0; i<8; i++){
        motion_model_[i][0] = xs[i];
        motion_model_[i][1] = ys[i];
        motion_model_[i][2] = cost[i];
    }
}

int DijkstracClass::twoD_to_oneD(const int x, const int y, const int width, const int height)
{
    int out= y*width + x;
    return out;
}



void DijkstracClass::explore_one_node(const array<int, 2> curr_xy){
    // move it into closed 
    all_nodes[curr_xy].node_state = 2;

    open_nodes.erase(curr_xy);

    // cout << curr_xy[0] << " " << curr_xy[1] << endl;

    for(auto mm : motion_model_){
        int n_x = curr_xy[0] + mm[0];
        int n_y = curr_xy[1] + mm[1];
        int edge_cost = mm[2];
        float heading = float( mm[3] );

        if(0<= n_x && n_x < map_width_ && 0<= n_y && n_y < map_height_){

            int grid_index = twoD_to_oneD(n_x, n_y, map_width_, map_height_);
            int obs_prob = int(grid_map[ grid_index ]);
            if( obs_prob != 100 ){
                array<int,2> n_node = {n_x, n_y};
                if(all_nodes[n_node].node_state == 0){
                    all_nodes[n_node].parent = curr_xy;
                    all_nodes[n_node].node_state = 1;
                    all_nodes[n_node].node_gcost = all_nodes[curr_xy].node_gcost + edge_cost;
                    all_nodes[n_node].heading = heading;
                    open_nodes.insert(n_node);
                } 
                if(all_nodes[n_node].node_state == 1 && all_nodes[n_node].parent != curr_xy)
                {
                    int old_g_cost = all_nodes[n_node].node_gcost;
                    int new_g_cost = all_nodes[curr_xy].node_gcost + edge_cost;
                    if(new_g_cost < old_g_cost)
                    {
                        all_nodes[n_node].parent = curr_xy;
                        all_nodes[n_node].node_gcost = new_g_cost;
                    }
                }
            }
        }
    }
}




vector< array<int, 2>> DijkstracClass::find_min_cost_nodes(){
    vector< array<int, 2>> out;
    int min_cost = std::numeric_limits<int>::max(); 

    // int num_opennodes = 0;
    // for(auto n : all_nodes){
    for(auto n : open_nodes){
        // array<int, 2> node = n.first;
        array<int, 2> node = n;
        // if(all_nodes[node].node_state == 1){
            int cost = all_nodes[node].node_gcost;
            if(cost < min_cost){
                min_cost = cost;
                out.clear();
                out.push_back(node);
            }
            if(cost == min_cost){
                out.push_back(node);
            }
            // num_opennodes ++;
        // }
    }
    // cout << "                  open: " << num_opennodes << endl;
    // cout << "                  open: " << open_nodes.size() << endl;
    return out;
}


void DijkstracClass::explore_one_ite(vector< std::array<int, 2>> active_nodes){
    for(vector<std::array<int, 2>>::iterator it = active_nodes.begin(); it != active_nodes.end(); ++it){
        explore_one_node(*it);
    }
}


bool DijkstracClass::search(){

    std::cout << "Dijkstra looking for path. ";// << std::endl;

    high_resolution_clock::time_point time1 = high_resolution_clock::now();
    high_resolution_clock::time_point time_temp;
    int count = 0;

    while(all_nodes[goal_xy_].node_state == 0){

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
        auto opennodes = find_min_cost_nodes();
        if(opennodes.size() == 0)
        {
            std::cout << "No valid path found. \nGetting the path closest to the goal." << std::endl;
            // find_min_FHcost_nodes(goal_xy);
            // return true;
            return false;
        }
        
        // regular Dijkstra exploration
        explore_one_ite( opennodes );

        count ++;
    }

    high_resolution_clock::time_point time2 = high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(time2-time1);

    std::cout << "Found the goal. Time used " << int(duration.count()/1000.0) << " ms." << std::endl;
    return true;
}


std::deque< DijkstracClass::pathPoint > DijkstracClass::get_path(){
    array<int, 2> get_path_curr_node = goal_xy_;
    
    while (path_.front().xy != start_xy_)
    {
        pathPoint pp;
        pp.xy = get_path_curr_node;
        // pp.xy[0] *= subsample_size;
        // pp.xy[1] *= subsample_size;

        // pp.heading = all_nodes[get_path_curr_node].heading;

        get_path_curr_node = all_nodes[get_path_curr_node].parent;
        
        path_.push_front( pp );
    }
    for(int i = 0; i<path_.size(); i++)
    {
        float dx = path_[i+1].xy[0] - path_[i].xy[0];
        float dy = path_[i+1].xy[1] - path_[i].xy[1];
        path_[i].heading = rectify( atan2(dy,dx) ) * 180.0 / M_PI;
    }
    path_[ path_.size()-1 ].heading = goal_angle_* 180.0 / M_PI;
    return path_;

}



float DijkstracClass::rectify( float a)
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


// void DijkstracClass::print_path(){
//     std::cout << "Find path:" << std::endl;
//     for(auto p : path){
//         std::cout << p[0] << " " << p[1] << std::endl;
//     }
// }


// void DijkstracClass::print_init_info(){

//     std::cout << "\nDijkstracClass start" 
//     << "\n\nmap size:  \nw: " << map_width << " h: " << map_height
//     << "\n\nStart location: \n" << start_xy[0] << " " << start_xy[1]
//     << "\n\nGoal location : \n" << goal_xy[0] << " " << goal_xy[1]
//     << "\n\nmotion model: \n" << motion_model[0][0] << "  " << motion_model[0][1]
//     << "\n" << motion_model[1][0] << "  " << motion_model[1][1]
//     << "\n" << motion_model[2][0] << "  " << motion_model[2][1]
//     << "\n" << motion_model[3][0] << "  " << motion_model[3][1]
//     << "\n" << motion_model[4][0] << "  " << motion_model[4][1]
//     << "\n" << motion_model[5][0] << "  " << motion_model[5][1]
//     << "\n" << motion_model[6][0] << "  " << motion_model[6][1]
//     << "\n" << motion_model[7][0] << "  " << motion_model[7][1]

//     << std::endl;

// }






// void DijkstracClass::draw_start_goal_on_map(){
//     cv::Vec3b color = {0,0,255} ;
//     map_for_view.at<cv::Vec3b>(start_xy[1], start_xy[0]) = color;
//     map_for_view.at<cv::Vec3b>(goal_xy[1], goal_xy[0]) = color;
// }











