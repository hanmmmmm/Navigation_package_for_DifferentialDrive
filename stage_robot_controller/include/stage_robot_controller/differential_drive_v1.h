#ifndef DIFFERENTIAL_DRIVE_V1_H
#define DIFFERENTIAL_DRIVE_V1_H

#include "ros/ros.h"
// #include "tf/transform_listener.h"
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <algorithm>
#include <atomic>
#include <vector>

#include "path_data.h"
// #include "../hybrid_a_star_module/hybrid_astar.h"

#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"

using std::array;
using std::cout;
using std::endl;
using std::string;
using std::vector;





class Diff_drive_v1_class : public pathPoint
{
private:

    struct robotPose
    {
        float x_meter, y_meter, x_pred, y_pred; // meter
        float yaw_rad, yaw_pred;    // radian
        float x_last, y_last, yaw_last;
    };

    float gene_cmdvel_time_interval_;

    robotPose robot_pose_;

    vector<pathPoint> path_;

    std::atomic<bool> FLAG_atom_local_goal_available;
    std::atomic<bool> FLAG_atom_local_path_in_use;
    std::atomic<bool> FLAG_atom_global_path_in_use;
    std::atomic<bool> FLAG_atom_local_map_in_use;
    std::atomic<bool> FLAG_atom_odom_in_use;


    geometry_msgs::Twist cmdvel_msg_;

    float map_size_;

    ros::Publisher cmd_vel_puber_ ;    
    ros::Subscriber odom_suber_ ;
    ros::Subscriber local_path_suber_ ;
    ros::Timer  periodic_cmdvel_;

    ros::NodeHandle nh_;


    float pose_pred_duration_; // seconds


    float rectify( float a);




public:
    Diff_drive_v1_class(const ros::NodeHandle nh_in);
    ~Diff_drive_v1_class();

    void odom_callback(const nav_msgs::Odometry::ConstPtr &msg);
    void local_path_callback(const nav_msgs::Path::ConstPtr &msg);
    void gene_cmdvel( const ros::TimerEvent &event );


};



#endif




