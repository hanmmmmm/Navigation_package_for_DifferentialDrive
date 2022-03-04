
#include "../include/global_planner/global_planner_node.h"

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <algorithm>
#include <atomic>
#include <thread>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

// #include <filesystem>

#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"

namespace pt = boost::property_tree;

globalHandle::globalHandle(const ros::NodeHandle nh_in_) : pathPoint(), nh_{nh_in_}
{
    cout << "globalHandle  init " << endl;

    FLAG_atom_global_goal_in_use.store(false);
    FLAG_atom_global_map_in_use.store(false);
    FLAG_atom_odom_in_use.store(false);
    FLAG_atom_global_goal_available.store(false);

    load_parameters();

    build_inflate_sample(inflation_.inflate_sample, inflation_.inflate_radius);

    map_puber_  = nh_.advertise<nav_msgs::OccupancyGrid>( map_published_topic_name_, 10);
    path_puber_ = nh_.advertise<nav_msgs::Path>( path_published_topic_name_ ,  10);

    suber_map_ = nh_.subscribe( map_subscribed_topic_name_ , 10, &globalHandle::map_callback,  this);
    suber_goal_= nh_.subscribe( goal_subscribed_topic_name_ , 10, &globalHandle::goal_callback, this);
    suber_odom_= nh_.subscribe( odom_subscribed_topic_name_ , 10, &globalHandle::odom_callback, this);

    periodic_path_planer_ = nh_.createTimer( ros::Duration(path_plan_time_interval_), &globalHandle::path_plan, this );

    cout << "globalHandle  init done" << endl;

}



void globalHandle::load_parameters()
{
    // char tmp[256];
    // getcwd(tmp, 256);

    // cout << tmp << endl;

    pt::ptree root;

    pt::read_json("/home/jf/robot_navigation_module_v1/src/global_planner/cfg/global_planner_cfg.json", root);

    path_finding_timeout_ms_ = root.get<int>("path_finding_timeout_ms_");
    path_plan_time_interval_ = root.get<float>("path_plan_time_interval_sec");

    pt::ptree inflate = root.get_child("inflation");

    inflation_.inflate_radius = inflate.get<int>("inflate_radius");

    pt::ptree topic_names = root.get_child("topic_names");

    map_subscribed_topic_name_ = topic_names.get<std::string>("map_subscribed_topic_name_");
    goal_subscribed_topic_name_ = topic_names.get<std::string>("goal_subscribed_topic_name_");
    odom_subscribed_topic_name_ = topic_names.get<std::string>("odom_subscribed_topic_name_");

    map_published_topic_name_ = topic_names.get<std::string>("map_published_topic_name_");
    path_published_topic_name_ = topic_names.get<std::string>("path_published_topic_name_");
}



void globalHandle::path_plan( const ros::TimerEvent &event )
{
    cout << "\nTimer Event: path_plan" << endl;

    // re-path-plan
    if ( FLAG_atom_global_goal_available.load() )
    {
        while( FLAG_atom_global_map_in_use.load() )
        {
            std::this_thread::yield();
        }
        FLAG_atom_global_map_in_use.store(true);
        
        // get current robot pose ------------------------------------------------
        while( FLAG_atom_odom_in_use.load() )
        {
            std::this_thread::yield();
        }
        FLAG_atom_odom_in_use.store(true);
        int robot_xy[2] = { robot_pose_.x_grid - map_info_.grid_origin_offset_x, robot_pose_.y_grid - map_info_.grid_origin_offset_y };
        FLAG_atom_odom_in_use.store(false);
        // -----------------------------------------------------------------------


        // get goal pose ---------------------------------------------------------
        while( FLAG_atom_global_goal_in_use.load() )
        {
            std::this_thread::yield();
        }
        FLAG_atom_global_goal_in_use.store(true);
        int goal_xy[2] = { global_goal_.x_grid - map_info_.grid_origin_offset_x, global_goal_.y_grid - map_info_.grid_origin_offset_y };

        if( map_info_.grid_map[ goal_xy[1] * map_info_.grid_width + goal_xy[0] ] == 100 )
        {
            // change the goal pose
            cout << "\nGoal is obstacle, exiting" << endl;
            // return;
        }

        FLAG_atom_global_goal_in_use.store(false);
        // -----------------------------------------------------------------------


        // path planning 
        bool new_robot_pose = robot_pose_.x_grid != robot_pose_.x_grid_prev  ||  robot_pose_.y_grid != robot_pose_.y_grid_prev;
        bool new_goal_pose = global_goal_.x_meter_last != global_goal_.x_meter ||
                            global_goal_.y_meter_last != global_goal_.y_meter ||
                            global_goal_.yaw_rad_last != global_goal_.yaw_rad;

        cout << "new_robot_pose " << new_robot_pose  << "   new_goal_pose " << new_goal_pose << endl;

        if(new_goal_pose || new_robot_pose)
        {
            path_finder_.setup( robot_xy, goal_xy, global_goal_.yaw_rad , map_info_.grid_map, map_info_.grid_height, map_info_.grid_width, path_finding_timeout_ms_ );
            bool find_path = path_finder_.search();
            if(find_path)
            {
                path_ = path_finder_.get_path();
                path_prev_ = path_;
                cout << "Found new path, steps  " << path_.size() << endl;
                // cout << "path_prev_ steps  " << path_prev_.size() << endl;
            }
            else
            {
                cout << "Path finding failed." << endl;
            }
            
            
        }
        else
        {
            path_ = path_prev_;
            cout << "Using previous path, size: " << path_.size() << endl;
        }
        robot_pose_.x_grid_prev = robot_pose_.x_grid;
        robot_pose_.y_grid_prev = robot_pose_.y_grid;

        if(new_goal_pose)
        {
            global_goal_.x_meter_last = global_goal_.x_meter;
            global_goal_.y_meter_last = global_goal_.y_meter;
            global_goal_.yaw_rad_last = global_goal_.yaw_rad;
        }

        FLAG_atom_global_map_in_use.store(false);

        

        // publish path
        nav_msgs::Path g_path;
        g_path.header.frame_id = map_info_.frameID;
        g_path.header.stamp = ros::Time::now();

        // g_path.poses.push_back();
        for(auto pp : path_)
        {
            int x = pp.xy[0] + map_info_.grid_origin_offset_x;
            int y = pp.xy[1] + map_info_.grid_origin_offset_y;
            float head = pp.heading * M_PI / 180.0;
            // cout << "path  [ " << x << ", " << y << ", " << head << " ]" << endl;
            float qz = sin(head/2.0) ;
            float qw = cos(head/2.0) ;
            geometry_msgs::PoseStamped pose;
            pose.header = g_path.header;
            pose.pose.position.x = float(x)*map_info_.resolution;// - robot_pose_.x_meter;
            pose.pose.position.y = float(y)*map_info_.resolution;// - robot_pose_.y_meter;
            pose.pose.orientation.w = qw;
            pose.pose.orientation.z = qz;
            g_path.poses.push_back( pose );
        }
        path_puber_.publish(  g_path);


        

    }
    else{
        cout << "Waiting for 1st goal " << endl;
    }

}


void globalHandle::map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    auto rostime = ros::Time::now();

    // cout << "\nReceived a new map, time: sec " << rostime.sec << endl;

    // if( FLAG_use_first_map && processed_map_count_ >= 1)
    // {
    //     return;
    // }

    // create the new message to be sent out
    nav_msgs::OccupancyGrid inflated_map;

    // filling most of the information by the original info
    inflated_map.header = msg->header;
    inflated_map.header.stamp = rostime;
    inflated_map.info = msg->info;
    inflated_map.info.map_load_time = rostime;
    inflated_map.data = msg->data; // it contains these states: -1, 0, 100  

    long position_counter = 0;
    // update the map, where the grid value is 100 (obstacle)
    // cout << "map inflation  " << endl;
    for (auto p : msg->data)
    {
        if(p > 1)
        {
            int y = position_counter / msg->info.width;
            int x = position_counter % msg->info.width;
            // cout << position_counter << "  x " << x << "  y " << y << endl;
            if( 0<x && x<msg->info.width  &&  0<y && y<msg->info.height)
            {
                inflate( inflated_map.data, x, y, msg->info.width, msg->info.height, inflation_.inflate_sample, inflation_.inflate_radius);
            }
        }
        position_counter ++;
    }
    // cout << "position_counter  " << position_counter  << endl;

    //publish out 
    map_puber_.publish(inflated_map);
    // processed_map_count_ ++;
    // cout << "processed_map_count_  " << processed_map_count_ << endl;


    while( FLAG_atom_global_map_in_use.load() )
    {
        std::this_thread::yield();
    }
    FLAG_atom_global_map_in_use.store(true);

    // update the grid map info, using the newest map message
    map_info_.grid_width = msg->info.width;
    map_info_.grid_height = msg->info.height;
    map_info_.resolution = msg->info.resolution;
    map_info_.frameID = msg->header.frame_id;
    map_info_.grid_map.resize( map_info_.grid_width * map_info_.grid_height );
    int x_offset = msg->info.origin.position.x / msg->info.resolution;
    int y_offset = msg->info.origin.position.y / msg->info.resolution;
    map_info_.grid_origin_offset_x = x_offset;
    map_info_.grid_origin_offset_y = y_offset;
    
    map_info_.grid_map = inflated_map.data;
    // std::copy();
    
    FLAG_atom_global_map_in_use.store(false);
}








void globalHandle::odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    yaw = rectify(yaw);

    while( FLAG_atom_odom_in_use.load() )
    {
        std::this_thread::yield();
    }
    FLAG_atom_odom_in_use.store(true);

    robot_pose_.robot_yaw_rad = yaw;
    robot_pose_.x_meter = msg->pose.pose.position.x;
    robot_pose_.y_meter = msg->pose.pose.position.y;
    robot_pose_.x_grid = int( robot_pose_.x_meter / map_info_.resolution );
    robot_pose_.y_grid = int( robot_pose_.y_meter / map_info_.resolution );
    // cout << robot_x << " "  << robot_y << " " << robot_yaw_rad << endl;

    FLAG_atom_odom_in_use.store(false);
}







void globalHandle::goal_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    tf::Quaternion q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    yaw = rectify(yaw);

    bool global_goal_ready = FLAG_atom_global_goal_in_use.load(std::memory_order_seq_cst);

    while ( FLAG_atom_global_goal_in_use.load() )
    {
        std::this_thread::yield();
    }

    FLAG_atom_global_goal_in_use.store( true );
    global_goal_.yaw_rad = yaw;
    global_goal_.x_meter = msg->pose.position.x;
    global_goal_.y_meter = msg->pose.position.y;
    global_goal_.x_grid = int( global_goal_.x_meter / map_info_.resolution );
    global_goal_.y_grid = int( global_goal_.y_meter / map_info_.resolution );
    
    cout << "\nNew global_goal  x:" << global_goal_.x_meter << " m, y:"  << global_goal_.y_meter << "m, heading:" << global_goal_.yaw_rad << " rad, grid: ";
    cout << global_goal_.x_grid << " , " << global_goal_.y_grid << endl;

    FLAG_atom_global_goal_in_use.store( false );

    if ( !FLAG_atom_global_goal_available.load() )
    {
        FLAG_atom_global_goal_available.store( true );
    }

    

}





int main(int argc, char **argv)
{
    ros::init(argc, argv, "global_planner_node");

    ros::NodeHandle n;

    globalHandle global_planner( n );

    // ros::spin();
    ros::AsyncSpinner s(4);
    s.start();

    ros::waitForShutdown();

    return 0;
}







