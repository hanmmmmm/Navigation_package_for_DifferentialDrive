
#include "../include/local_planner/local_planner_node.h"

#include "ros/ros.h"
#include "tf/transform_listener.h"
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <algorithm>
#include <atomic>
#include <future>
#include <thread>
#include <chrono>

#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/Odometry.h"




scanListener::scanListener(const ros::NodeHandle nh_in)
: nh_{nh_in}
{
    cout << "scanListener  init " << endl;

    cout << "setup variables" << endl;
    
    FLAG_atom_local_goal_available.store(false);
    FLAG_atom_local_goal_in_use.store(false);
    FLAG_atom_global_path_in_use.store(false);
    FLAG_atom_local_map_in_use.store(false);
    FLAG_atom_odom_in_use.store(false);
    

    cout << "setup map_info_" << endl;

    map_info_.frameID = "/base_link";
    map_info_.resolution = 0.1;
    map_info_.meter_width = 2.0;
    map_info_.meter_height = 2.0;
    map_info_.grid_width = map_info_.meter_width / map_info_.resolution;
    map_info_.grid_height = map_info_.meter_height / map_info_.resolution;
    map_info_.grid_map.resize( map_info_.grid_height * map_info_.grid_width );

    inflation_.inflate_radius = 3;
    build_inflate_sample(inflation_.inflate_sample, inflation_.inflate_radius);

    path_plan_time_interval = 0.2;

    path_plan_timeout_ms_ = 500;

    cout << "sub and pub" << endl;

    local_map_puber_ = nh_.advertise<nav_msgs::OccupancyGrid>("local_map", 10);
    local_path_puber_ = nh_.advertise<nav_msgs::Path>("local_path", 10);

    scan_suber_ = nh_.subscribe("base_scan", 10, &scanListener::scan_callback, this);
    odom_suber_ = nh_.subscribe("odom", 10, &scanListener::odom_callback, this);
    global_path_suber_ = nh_.subscribe( "global_path", 10, &scanListener::global_path_callback, this );

    periodic_path_planer_ = nh_.createTimer( ros::Duration(path_plan_time_interval), &scanListener::path_plan, this );


    // get_tf_laser_to_base(laser_base_tf);

    

    std::cout << "scanListener  init done" << std::endl;

}


void scanListener::scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    nav_msgs::OccupancyGrid local_grid;
    local_grid.header.frame_id = map_info_.frameID;
    local_grid.header.stamp = ros::Time::now();

    nav_msgs::MapMetaData metadata;
    metadata.map_load_time = ros::Time::now();
    metadata.width = map_info_.grid_width;
    metadata.height = map_info_.grid_height;
    metadata.origin.position.x = -map_info_.meter_width/2;
    metadata.origin.position.y = -map_info_.meter_height/2;
    metadata.resolution = map_info_.resolution;
    local_grid.info = metadata;

    std::fill(map_info_.grid_map.begin(), map_info_.grid_map.end(), 0);

    float point_angle = M_PI; // msg->angle_min;
    const float angle_inc = msg->angle_increment;

    for (auto p : msg->ranges)
    {
        point_angle += angle_inc;
        point_angle = rectify(point_angle);
        
        if(p < 10.0 && p > 0.1)
        {
            float p_x_meter = p*cos(point_angle) + map_info_.meter_width/2.0 ;
            float p_y_meter = p*sin(point_angle) + map_info_.meter_height/2.0 ;
            int p_x_grid = p_x_meter / map_info_.resolution;
            int p_y_grid = p_y_meter / map_info_.resolution;
            // std::cout << p_x_grid << " " << p_y_grid << std::endl;
            // update gridmap
            if(0< p_x_grid && p_x_grid < map_info_.grid_width-1 && 0 < p_y_grid && p_y_grid < map_info_.grid_height-1)
            {
                inflate(map_info_.grid_map,p_x_grid,p_y_grid,map_info_.grid_width, map_info_.grid_height, inflation_.inflate_sample, inflation_.inflate_radius );
            }
        }
    }

    // publish
    local_grid.data = map_info_.grid_map;
    local_map_puber_.publish(local_grid);
}


void scanListener::odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
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

    FLAG_atom_odom_in_use.store(false);
    // std::cout << robot_x << " "  << robot_y << " " << robot_yaw_rad << std::endl;
}


void scanListener::global_path_callback(const nav_msgs::Path::ConstPtr &msg)
{


    while( FLAG_atom_odom_in_use.load() )
    {
        std::this_thread::yield();
    }
    FLAG_atom_odom_in_use.store(true);
    float robot_x = robot_pose_.x_meter;
    float robot_y = robot_pose_.y_meter; 
    float robot_w = robot_pose_.robot_yaw_rad;
    FLAG_atom_odom_in_use.store(false);


    while( FLAG_atom_global_path_in_use.load() )
    {
        std::this_thread::yield();
    }
    FLAG_atom_global_path_in_use.store(true);
    int path_point_count = 0;
    global_path_.clear();
    for(auto pp : msg->poses)
    {
        tf::Quaternion q(
        pp.pose.orientation.x,
        pp.pose.orientation.y,
        pp.pose.orientation.z,
        pp.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        yaw = rectify(yaw);

        float ppx = pp.pose.position.x;
        float ppy = pp.pose.position.y;
        float ppyaw = yaw;
        
        float dx = robot_x - ppx;
        float dy = robot_y - ppy;
        float dd = sqrt(dx*dx + dy*dy);
        float beta = rectify( atan2(dy,dx) ) - M_PI - robot_w;

        float x_local = dd * cos(beta);
        float y_local = dd * sin(beta);
        float yaw_local = ppyaw - robot_w;

        pathPoint point;
        point.heading = yaw_local;
        array<float,2>pointxy {x_local, y_local};
        point.xy = pointxy;

        // cout << path_point_count << ": ";
        // cout << ppx << " ' " << ppy << " ' " << ppyaw*180.0/M_PI << " ' " << robot_x << " ' "  << robot_y << " ' "  << robot_w*180.0/M_PI << " ' " ;
        // cout << "beta " << beta*180.0/M_PI ;
        // cout << "  |  " << x_local << " "  << y_local << " " << yaw_local << std::endl;
        // path_point_count ++;
        // cout << gpp.xy[0] << " " << gpp.xy[1] << " " << gpp.heading << endl;

        // global_path_.push_back(pathPoint(x_local, y_local, yaw_local));
        global_path_.push_back( point );        
    }

    // for(auto gpp : global_path_)
    // {
    //     cout << gpp.xy[0] << " " << gpp.xy[1] << " " << gpp.heading << endl;
    // }

    cout << "global_path_  size: " << global_path_.size() << endl;
    FLAG_atom_global_path_in_use.store(false);

}



void scanListener::path_plan( const ros::TimerEvent &event )
{
    cout << "\npath_plan :: TimerEvent " << endl;

    while( FLAG_atom_global_path_in_use.load() )
    {
        std::this_thread::yield();
    }
    FLAG_atom_global_path_in_use.store(true);

    float map_width_temp = map_info_.meter_width;
    float map_height_temp = map_info_.meter_height;

    // cout << "path_plan :: got map size " << endl;

    nav_msgs::Path local_path;
    local_path.header.frame_id = "base_link";
    local_path.header.stamp = ros::Time::now();

    int path_point_count = 0;

    // cout << "path_plan :: built path obj" << endl;

    // cout << "path_plan :: global_path_ size: " << global_path_.size() << endl;

    if(global_path_.size() >1)
    {
        
        for(auto point : global_path_)
        {
            float x_local = point.xy[0];
            float y_local = point.xy[1];
            float yaw_local = point.heading;
            // cout << "path_plan :: getting points in global path: " << x_local << " " << y_local << " " << yaw_local << endl;

            // std::cout << path_point_count << ": " << x_local << " "  << y_local << " " << yaw_local << std::endl;
            path_point_count ++;
                
            if( -map_width_temp/2.0 <x_local && x_local < map_width_temp/2.0  && -map_height_temp/2.0 <y_local && y_local < map_height_temp/2.0  )
            {
                local_goal_.x_meter = x_local + map_info_.meter_width/2;
                local_goal_.y_meter = y_local + map_info_.meter_height/2;

                local_goal_.x_grid = local_goal_.x_meter / map_info_.resolution;
                local_goal_.y_grid = local_goal_.y_meter / map_info_.resolution;

                local_goal_.yaw_rad = yaw_local;

                // cout << "path_plan :: valid points : " <<local_goal_.x_meter << " " << local_goal_.y_meter << " " << local_goal_.yaw_rad << endl;
            }
            else
            {
                break;
            }


        }
    }

    FLAG_atom_global_path_in_use.store(false);

    if( global_path_.size() <=1 )
    {
        cout << "Localplanner::path_plan: Received invalid global path" << endl;
        return;
    }
        

    int startnode[2] = { map_info_.grid_width/2, map_info_.grid_height/2 };
    float startangle = 0.0;


    array<float,3> goalpose= {local_goal_.x_meter, local_goal_.y_meter , local_goal_.yaw_rad};

    cout << "path_plan :: goalpose: " << goalpose[0] << " " << goalpose[1] << " " << goalpose[2] << endl;

    path_finder_.setup( path_plan_timeout_ms_, startnode, startangle, goalpose, map_info_.meter_width, map_info_.meter_height, map_info_.grid_width, map_info_.grid_height, map_info_.grid_map );
    bool found_path = path_finder_.search();
    if( ! found_path )
    {
        return;
    }

    path_ = path_finder_.get_path();

    // // local_path_puber_.publish(local_path);
    // // publish path
    nav_msgs::Path g_path;
    g_path.header.frame_id = map_info_.frameID;
    g_path.header.stamp = ros::Time::now();

    // g_path.poses.push_back();
    for(auto pp : path_)
    {
        float x = pp.xy[0] ;//+ map_info_.grid_origin_offset_x;
        float y = pp.xy[1] ;//+ map_info_.grid_origin_offset_y;
        float head = pp.heading;// * M_PI / 180.0;
        // cout << "path  " << x << " " << y << " " << head << endl;
        float qz = sin(head/2.0) ;
        float qw = cos(head/2.0) ;
        geometry_msgs::PoseStamped pose;
        pose.header = g_path.header;
        pose.pose.position.x = x - map_info_.meter_width/2; //float(x)*map_info_.resolution;// - robot_pose_.x_meter;
        pose.pose.position.y = y - map_info_.meter_height/2; //float(y)*map_info_.resolution;// - robot_pose_.y_meter;
        pose.pose.orientation.w = qw;
        pose.pose.orientation.z = qz;
        g_path.poses.push_back( pose );

        cout << std::fixed << std::setprecision(3) << "path_plan :: path point: " << pose.pose.position.x << " " << pose.pose.position.y << " " << head << endl;

    }
    local_path_puber_.publish(  g_path );
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "local_planner_node");
    ros::NodeHandle n;
    
    scanListener listener( n );
    
    ros::AsyncSpinner s(4);
    s.start();

    ros::waitForShutdown();

    return 0;
}
