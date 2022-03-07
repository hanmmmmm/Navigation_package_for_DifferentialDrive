#ifndef GLOBAL_PLANNER_NODE_H
#define GLOBAL_PLANNER_NODE_H

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"

// #include "../A_star_module/astar.h"
// #include "../Dijkstra_module/dijkstra.h"

#include "../planner_module/planner.h"

#include "path_data.h"

#include <algorithm>
#include <string>
#include <atomic>
#include <vector>

using std::array;
using std::cout;
using std::endl;
using std::string;
using std::vector;

class globalHandle : public pathPoint
{
private:

    struct globalGoalInfo
    {
        float x_meter, y_meter, yaw_rad;
        int x_grid, y_grid;

        float x_meter_last, y_meter_last, yaw_rad_last;
    };

    struct globalMapInfo
    {
        float resolution ; // Resolution of the grid [m/cell].
        float meter_width;       // meter
        float meter_height;      // meter
        int grid_width;          // grid_wise
        int grid_height;         // grid_wise
        int array_size;          // grid_wise
        

        int grid_origin_offset_x;
        int grid_origin_offset_y;

        std::string frameID;          // grid_wise
        std::vector<int8_t> grid_map; // the main local_map
        
    };

    struct inflationInfo
    {
        std::vector<std::vector<int8_t>> inflate_sample;
        int inflate_radius ;  // grid_wise
    };

    struct robotPose
    {
        float x_meter, y_meter; // meter
        float robot_yaw_rad;    // radian
        int x_grid, y_grid;
        int x_grid_prev, y_grid_prev;
    };

    float path_plan_time_interval_;

    int path_finding_timeout_ms_;

    Planner_class path_finder_;

    int planner_type_; 

    // AstarClass path_finder_;
    // DijkstracClass path_finder_;
    // DijkstracClass* path_finder_ = new DijkstracClass();

    // void* path_finder_;

    //  member variables

    std::deque<pathPoint> path_;
    std::deque<pathPoint> path_prev_;

    globalGoalInfo global_goal_;

    globalMapInfo map_info_;

    robotPose robot_pose_;

    inflationInfo inflation_;

    bool FLAG_use_first_map_;
    unsigned int processed_map_count_;

    std::atomic<bool> FLAG_atom_global_goal_available;
    std::atomic<bool> FLAG_atom_global_goal_in_use;
    std::atomic<bool> FLAG_atom_global_map_in_use;
    std::atomic<bool> FLAG_atom_odom_in_use;

    std::string map_subscribed_topic_name_;
    std::string goal_subscribed_topic_name_;
    std::string odom_subscribed_topic_name_;
    std::string map_published_topic_name_;
    std::string path_published_topic_name_;

    ros::Publisher map_puber_;
    ros::Publisher path_puber_;

    ros::Subscriber suber_map_;
    ros::Subscriber suber_goal_;
    ros::Subscriber suber_odom_;

    ros::Timer  periodic_path_planer_;

    ros::NodeHandle nh_;


    //  member functions
    // void get_tf_laser_to_base(std::array<float, 3> &the_tf);

    void load_parameters();

    float rectify(float angle);

    void build_inflate_sample(std::vector<std::vector<int8_t>> &inflate_sample_target, const int radius);

    void inflate(std::vector<int8_t> &grids, const int x, const int y, const int map_width, const int map_height, const std::vector<std::vector<int8_t>> &inflate_sample_in, const int radius);

    void path_plan( const ros::TimerEvent &event );

public:
    // globalHandle(const float width_in_, const float height_in_, const std::string frameID_in_, ros::Publisher pub, ros::Publisher path_pub);
    globalHandle(const ros::NodeHandle nh_in_);

    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    void odom_callback(const nav_msgs::Odometry::ConstPtr &msg);
    void goal_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
};

#endif