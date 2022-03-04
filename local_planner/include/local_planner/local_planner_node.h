#ifndef LOCAL_PLANNER_NODE_H
#define LOCAL_PLANNER_NODE_H

#include "ros/ros.h"
// #include "tf/transform_listener.h"
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <algorithm>
#include <atomic>
#include <deque>

#include "path_data.h"
#include "../hybrid_a_star_module/hybrid_astar.h"

#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"

using std::array;
using std::cout;
using std::endl;
using std::string;
using std::vector;



class Local_planner_class : public pathPoint
{
private:

    Hybrid_astar_class path_finder_;

    struct localGoalInfo
    {
        float x_meter, y_meter, yaw_rad;
        int x_grid, y_grid;
    };

    struct localMapInfo
    {
        float resolution       ; // Resolution of the grid [m/cell].
        float meter_width;       // meter
        float meter_height;      // meter
        int grid_width;          // grid_wise
        int grid_height;         // grid_wise
        int array_size;          // grid_wise
        

        int grid_origin_offset_x;
        int grid_origin_offset_y;

        string frameID;          // grid_wise
        vector<int8_t> grid_map; // the main local_map
        
    };

    struct inflationInfo
    {
        vector<vector<int8_t>> inflate_sample;
        int inflate_radius = 5;  // grid_wise
    };

    struct robotPose
    {
        float x_meter, y_meter; // meter
        float robot_yaw_rad;    // radian
        int x_grid, y_grid;
        int x_grid_prev, y_grid_prev;
    };

    float path_plan_time_interval_;

    std::deque<pathPoint> path_;
    std::deque<pathPoint> path_prev_;

    vector<pathPoint> global_path_;

    localGoalInfo local_goal_;

    localMapInfo map_info_;

    robotPose robot_pose_;

    inflationInfo inflation_;

    int path_plan_timeout_ms_;

    std::string scan_topic_name_;
    std::string odom_topic_name_; 
    std::string global_path_topic_name_; 

    std::string local_path_topic_name_; 
    std::string local_map_topic_name_; 

    // std::deque< pathPoint > path_;

    //  member variables

    std::atomic<bool> FLAG_atom_local_goal_available;
    std::atomic<bool> FLAG_atom_local_goal_in_use;
    std::atomic<bool> FLAG_atom_global_path_in_use;
    std::atomic<bool> FLAG_atom_local_map_in_use;
    std::atomic<bool> FLAG_atom_odom_in_use;

    ros::Publisher local_map_puber_ ;
    ros::Publisher local_path_puber_ ;
    
    ros::Subscriber scan_suber_ ;
    ros::Subscriber odom_suber_ ;
    ros::Subscriber global_path_suber_ ;

    ros::Timer  periodic_path_planer_;

    ros::NodeHandle nh_;


    //  member functions

    void load_parameters();
    // void get_tf_laser_to_base(array<float, 3> &the_tf);

    float rectify( float angle);

    void build_inflate_sample( vector< vector<int8_t> >& inflate_sample_target, const int radius );

    void inflate(vector<int8_t>& grids, const int x, const int y, const int map_width, const int map_height, const vector<vector<int8_t>>& inflate_sample_in , const int radius  );


public:



    Local_planner_class(const ros::NodeHandle nh_in);
    ~Local_planner_class(){};

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg);
    void odom_callback(const nav_msgs::Odometry::ConstPtr &msg);
    void global_path_callback(const nav_msgs::Path::ConstPtr &msg);
    void path_plan( const ros::TimerEvent &event );

};






#endif



