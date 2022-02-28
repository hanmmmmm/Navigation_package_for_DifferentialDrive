
#include <iostream>
#include <string>
#include <stdexcept>
// #include <opencv2/core.hpp>
// #include <opencv2/highgui.hpp>
// #include <opencv2/imgproc.hpp>
#include <limits>
#include <math.h>
#include <iomanip>
// #include "utils/img_io.h"
#include "utils/bresenham.h"
#include "utils/math_tools.h"

#include "hybrid_astar.h"

using std::chrono::high_resolution_clock;

Hybrid_astar_class::Hybrid_astar_class() : pathPoint()
{

}

Hybrid_astar_class::~Hybrid_astar_class() 
{

}


void Hybrid_astar_class::setup(const int timeoutms,
    int startnode[], float s_angle, array<float,3> goalpose,
    const float map_width_meter, const float map_height_meter, 
    const int map_width_grid, const int map_height_grid, 
    const std::vector<int8_t>& map )
{
    // prepare the maps 

    time_out_ms_ = timeoutms;

    map_grid = map;

    path_.clear();
    all_grids_.clear();

    FLAG_reach_goal_ = false;

    fine_map_height_ = map_height_meter;
    fine_map_width_  = map_width_meter;

    step_length_ = 0.2;
    turning_raius_ = 0.5;
    turning_angle_ = step_length_ / turning_raius_;

    grid_map_width_ = map_width_grid;   
    grid_map_height_ = map_height_grid; 

    start_pose_[0] = map_width_meter/2;
    start_pose_[1] = map_height_meter/2;
    start_pose_[2] = s_angle;

    start_grid_ = { startnode[0], startnode[1], 0 };

    all_grids_[start_grid_].fine_pose = start_pose_;
    all_grids_[start_grid_].parent = start_grid_;
    all_grids_[start_grid_].state = 1;
    all_grids_[start_grid_].steer_type = 1;

    goal_pose_[0] = goalpose[0];
    goal_pose_[1] = goalpose[1];
    goal_pose_[2] = goalpose[2];

    if(goal_pose_[0] <0)
    {
        goal_pose_[0] = 0;
    }
    else if(goal_pose_[0] > map_width_meter)
    {
        goal_pose_[0] = map_width_meter;
    }

    if(goal_pose_[1] <0)
    {
        goal_pose_[1] = 0;
    }
    else if(goal_pose_[1] > map_height_meter)
    {
        goal_pose_[1] = map_height_meter;
    }

    if(goal_pose_[2] <0)
    {
        goal_pose_[2] = 0;
    }
    else if(goal_pose_[2] > 2*M_PI)
    {
        goal_pose_[2] = 2*M_PI;
    }

    goal_grid_ = fine_to_grid(goal_pose_, fine_ratio_, angle_resolution_);

    goal_tolerance.max_distance_error = 0.2;
    goal_tolerance.max_heading_error = 0.8;

    build_motion_model();

    print_init_info();
}







void Hybrid_astar_class::build_motion_model()
{
    float raius = turning_raius_;
    float angle = turning_angle_;
    // std::cout << "build_motion_model - turning_raius_: " << raius << std::endl;
    // std::cout << "build_motion_model - turning_angle_: " << angle << std::endl;

    float unit_dx = raius * sin(angle);
    float unit_dy = raius * (cos(angle) - 1);

    std::vector<float> temp = {unit_dx, -1 * unit_dy, angle, 0};
    motion_model_.push_back(temp);

    temp = {step_length_, 0, 0, 0};
    motion_model_.push_back(temp);

    temp = {unit_dx, unit_dy, float(M_PI * 2 - angle), 0};
    motion_model_.push_back(temp);

    temp = {-1 * unit_dx, unit_dy, angle, 1};
    motion_model_.push_back(temp);

    temp = {-1 * step_length_, 0, float(M_PI * 2 - angle), 1};
    motion_model_.push_back(temp);

    temp = {-1 * unit_dx, -1 * unit_dy, angle, 1};
    motion_model_.push_back(temp);
}




void Hybrid_astar_class::explore_one_node(std::array<float, 3> curr_pose, std::array<int, 3> curr_grid)
{
    all_grids_[curr_grid].state = 2;
    float curr_theta = curr_pose[2];
    float cos_theta = cos(curr_theta);
    float sin_theta = sin(curr_theta);

    // cout << "explore_one_node:: " << all_grids_[curr_grid].fine_pose[0] << " " << all_grids_[curr_grid].fine_pose[1] << " "
    //  << all_grids_[curr_grid].fine_pose[2] << endl;

    int count = 0;
    for (auto mm : motion_model_)
    {
        float dx = cos_theta * mm[0] + sin_theta * mm[1];
        float dy = sin_theta * mm[0] + cos_theta * mm[1];
        float nb_x = curr_pose[0] + dx;
        float nb_y = curr_pose[1] + dy;

        // cout << "explore_one_node:: " << nb_x << " " << nb_y << " " << curr_theta + mm[2] << endl;

        if (0 <= nb_x && nb_x < fine_map_width_ && 0 <= nb_y && nb_y < fine_map_height_)
        {
            int nb_x_grid = nb_x / fine_ratio_;
            int nb_y_grid = nb_y / fine_ratio_;
            int nb_grid_index = twoD_to_oneD(nb_x_grid, nb_y_grid, grid_map_width_, grid_map_height_);
            int obs_prob = int(map_grid[ nb_grid_index ]);
            if( obs_prob != 100 )
            { // if not obstacle

                float edge_cost = step_length_;
                int is_reversing = int(mm[3]);
                if (is_reversing)
                    {edge_cost *= 1.05;}

                int nb_steer = count;
                // int parent_steer = grid_status[curr_grid][3];
                int parent_steer = all_grids_[curr_grid].steer_type;

                float nb_a = curr_theta + mm[2]; // angle change in this step
                nb_a = rectify_angle_rad(nb_a);

                const std::array<float, 3> nb_pose = {nb_x, nb_y, nb_a};
                const std::array<int, 3> nb_grid = fine_to_grid(nb_pose, fine_ratio_, angle_resolution_);

                // if (grid_parent.count(nb_grid) == 0)
                if (all_grids_.count(nb_grid) == 0)
                {
                    all_grids_[nb_grid].fine_pose = nb_pose;
                    all_grids_[nb_grid].parent = curr_grid;
                    all_grids_[nb_grid].state = 1;
                    all_grids_[nb_grid].steer_type = count;
                    float gcost = all_grids_[curr_grid].gcost  + edge_cost;
                    all_grids_[nb_grid].gcost = gcost;

                    float hcost = compute_h_cost_Euclidean(nb_pose, goal_pose_);

                    // hcost = improve_hcost(nb_pose, hcost, obstacle_scan_range, nb_steer==parent_steer);

                    all_grids_[nb_grid].fcost = gcost + hcost ;
                }
                else{
                    if( all_grids_[nb_grid].state==1  ||  (all_grids_[nb_grid].state==2 && all_grids_[nb_grid].parent==curr_grid ) ){
                        float old_cost = all_grids_[nb_grid].fcost;
                        float gcost = all_grids_[curr_grid].gcost + edge_cost;
                        all_grids_[nb_grid].gcost = gcost;

                        float hcost = compute_h_cost_Euclidean(nb_pose, goal_pose_);
                        // hcost = improve_hcost(nb_pose, hcost, obstacle_scan_range, nb_steer==parent_steer);
                        float fcost = gcost + hcost;
                        
                        if(fcost < old_cost){
                            all_grids_[nb_grid].fine_pose = nb_pose;
                            all_grids_[nb_grid].parent = curr_grid;
                            all_grids_[nb_grid].state = 1;
                            all_grids_[nb_grid].steer_type = count;
                            all_grids_[nb_grid].fcost = fcost;
                            all_grids_[nb_grid].gcost = gcost;
                        }
                    }
                }

                check_if_reach_goal(nb_pose, goal_pose_, curr_grid);
                
            }
        }
        count++;
    }
}


int Hybrid_astar_class::twoD_to_oneD(const int x, const int y, const int width, const int height)
{
    int out= y*width + x;
    return out;
}


std::vector<std::array<int, 3>> Hybrid_astar_class::find_min_cost_nodes()
{
    // std::cout << "find_min_cost_nodes" << std::endl;
    std::vector<std::array<int, 3>> out;
    float min_cost = std::numeric_limits<float>::max();
    
    // for (auto n : grid_status) // if a node exist in node_parent
    for (auto n : all_grids_) 
    {
        std::array<int, 3> node = n.first;
        if (n.second.state == 1)  // if state == open
        {
            float cost = n.second.fcost;
            if (cost < min_cost)
            {
                min_cost = cost;
                out.clear();
                out.push_back(node);
            }
            else if (cost == min_cost)
            {
                out.push_back(node);
            }
            
        }

    }

    // cout << "min_cost  " << min_cost << endl;

    return out;
}




void Hybrid_astar_class::explore_one_ite(std::vector<std::array<int, 3>> active_nodes)
{
    for (std::vector<std::array<int, 3>>::iterator it = active_nodes.begin(); it != active_nodes.end(); ++it)
    {
        // auto fine_pose = grid_fine_pose[*it];
        auto fine_pose = all_grids_[*it].fine_pose;
        explore_one_node(fine_pose, *it);
    }


    
}





bool Hybrid_astar_class::search()
{
    std::cout << "Looking for path " << std::endl;

    high_resolution_clock::time_point time1 = high_resolution_clock::now();

    int search_count = 0;
    while (!FLAG_reach_goal_)
    {
        explore_one_ite( find_min_cost_nodes() );
        // cout << "search_count " << search_count << endl;
        search_count ++;

        high_resolution_clock::time_point time2 = high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(time2-time1);

        if( int(duration.count()/1000.0) >= time_out_ms_ )
        {
            cout << "search failed, timeout" << std::endl;
            return false;
        }


        // if(search_count < 3)
        // {
        //     for (auto n : all_grids_) 
        //     {
        //     cout << std::fixed << std::setprecision(3)  << "all grids: ["<< n.first[0]<< " " << n.first[1]<< " " << n.first[2]<< "]  g: " << n.second.gcost << " f: " << n.second.fcost << " ; ["
        //         << n.second.fine_pose[0] << " " << n.second.fine_pose[1] << " " << n.second.fine_pose[2] << "] "
        //         << n.second.state << " ["  << n.second.parent[0] << " "  << n.second.parent[1] << "] " << endl;
        //     }
        // }
    }

    std::cout << "Found the goal" << std::endl;
    return true;
}




std::deque< pathPoint >  Hybrid_astar_class::get_path()
{
    std::array<int, 3> get_path_curr_node = fine_to_grid(close_goal_pose_, fine_ratio_, angle_resolution_);
    // std::cout << "get_path" << std::endl;
    
    while (get_path_curr_node != start_grid_)
    {
        // cout << "Hybrid_astar_class::get_path: " << all_grids_[get_path_curr_node].fine_pose[0] << " " << all_grids_[get_path_curr_node].fine_pose[1] << " " << all_grids_[get_path_curr_node].fine_pose[2] << endl;
        pathPoint point;
        point.heading = all_grids_[get_path_curr_node].fine_pose[2];
        point.xy[0] = all_grids_[get_path_curr_node].fine_pose[0];
        point.xy[1] = all_grids_[get_path_curr_node].fine_pose[1];
        path_.push_front( point );
        
        get_path_curr_node = all_grids_[get_path_curr_node].parent;
    }

    return path_;
}


void Hybrid_astar_class::check_if_reach_goal(array<float, 3> node_pose,array<float, 3> in_goal_pose, array<int, 3> in_curr_grid){
    if (compute_h_cost_Euclidean(node_pose, in_goal_pose) < goal_tolerance.max_distance_error)
    {
        if (abs(node_pose[2] - in_goal_pose[2]) < goal_tolerance.max_heading_error  )
        {
            FLAG_reach_goal_ = true;
            close_goal_angle_ = node_pose[2];
            close_goal_pose_ = node_pose;
            all_grids_[goal_grid_].parent = in_curr_grid;
            // grid_parent[goal_grid] = in_curr_grid;
            all_grids_[goal_grid_].fine_pose = node_pose;
            // grid_fine_pose[goal_grid] = node_pose;
            all_grids_[goal_grid_].state = 2;
            // grid_status[goal_grid][2] = 2;
        }
    }
}

