
#include "../include/global_planner/global_planner_node.h"

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <algorithm>
#include <atomic>
#include <thread>


#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"




globalHandle::globalHandle(const float width_in_, const float height_in_, const std::string frameID_in_, ros::Publisher pub1, ros::Publisher pub2): pathPoint()
//: map_info_.meter_width{width_in_}, meter_height{height_in_}, frameID{frameID_in_}, puber{pub}
{
    FLAG_atom_global_goal_in_use.store(false);
    FLAG_atom_global_map_in_use.store(false);
    FLAG_atom_odom_in_use.store(false);
    FLAG_atom_global_goal_available.store(false);

    FLAG_use_first_map_ = true;
    processed_map_count_ = 0;

    map_info_.meter_width = width_in_;
    map_info_.meter_height = height_in_; 
    map_info_.frameID = frameID_in_;
    map_puber_ = pub1;
    path_puber_ = pub2;

    map_info_.grid_width = map_info_.meter_width / map_info_.resolution;
    map_info_.grid_height = map_info_.meter_height / map_info_.resolution;
    map_info_.grid_map.resize(map_info_.grid_width * map_info_.grid_height);
    map_info_.array_size = map_info_.grid_width * map_info_.grid_height;

    cout << "globalHandle  init " << endl;
    cout << map_info_.grid_width << " "  << map_info_.grid_height << " " << map_info_.array_size << endl;

    // get_tf_laser_to_base(laser_base_tf);

    build_inflate_sample(map_info_.inflate_sample, map_info_.inflate_radius);

    cout << "globalHandle  init done" << endl;

}



void globalHandle::map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    auto rostime = ros::Time::now();

    cout << "\nReceived a new map, time: sec " << rostime.sec << endl;

    // if( FLAG_use_first_map && processed_map_count_ >= 1)
    // {
    //     return;
    // }

    while( FLAG_atom_global_map_in_use.load() )
    {
        std::this_thread::yield();
    }
    FLAG_atom_global_map_in_use.store(true);

    // update the grid map info, using the newest map message
    map_info_.grid_width = msg->info.width;
    map_info_.grid_height = msg->info.height;
    map_info_.resolution = msg->info.resolution;
    int x_offset = msg->info.origin.position.x / msg->info.resolution;
    int y_offset = msg->info.origin.position.y / msg->info.resolution;
    
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
    for (auto p : msg->data)
    {
        if(p==100)
        {
            int y = position_counter / map_info_.grid_width;
            int x = position_counter % map_info_.grid_width;
            // cout << position_counter << "  x " << x << "  y " << y << endl;
            if( 0<x && x<map_info_.grid_width  &&  0<y && y<map_info_.grid_height)
            {
                inflate( inflated_map.data, x, y, map_info_.inflate_sample, map_info_.inflate_radius);
            }
        }
        position_counter ++;
    }

    //publish out 
    map_puber_.publish(inflated_map);
    processed_map_count_ ++;
    // cout << "processed_map_count_  " << processed_map_count_ << endl;

    // re-path-plan
    if ( FLAG_atom_global_goal_available.load() )
    {
        // get current robot pose
        while( FLAG_atom_odom_in_use.load() )
        {
            std::this_thread::yield();
        }
        FLAG_atom_odom_in_use.store(true);
        int robot_xy[2] = { robot_pose_.x_grid - x_offset, robot_pose_.y_grid - y_offset };
        FLAG_atom_odom_in_use.store(false);

        // get goal pose
        while( FLAG_atom_global_goal_in_use.load() )
        {
            std::this_thread::yield();
        }
        FLAG_atom_global_goal_in_use.store(true);
        int goal_xy[2] = { global_goal_.x_grid - x_offset, global_goal_.y_grid - y_offset };

        if( inflated_map.data[ goal_xy[1]*inflated_map.info.width + goal_xy[0] ] == 100 )
        {
            // change the goal pose
            cout << "\nGoal is obstacle, exiting" << endl;
        }

        FLAG_atom_global_goal_in_use.store(false);

        // path planning 
        path_finder_.setup( robot_xy, goal_xy, inflated_map.data, map_info_.grid_height, map_info_.grid_width );
        path_finder_.search();
        path = path_finder_.get_path();

        // publish path
        nav_msgs::Path g_path;
        g_path.header.frame_id = map_info_.frameID;
        g_path.header.stamp = ros::Time::now();

        // g_path.poses.push_back();
        for(auto pp : path)
        {
            int x = pp.xy[0] + x_offset;
            int y = pp.xy[1] + y_offset;
            float head = pp.heading * M_PI / 180.0;
            // cout << "path  " << x << " " << y << " " << head << endl;
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
    ros::init(argc, argv, "global_handle_node");
    ros::NodeHandle n;

    ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>("global_map_inflated", 10);

    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("global_path", 10);

    globalHandle listener(10, 10, "map", map_pub, path_pub);

    ros::Subscriber sub1 = n.subscribe("map", 10, &globalHandle::map_callback, &listener);

    ros::Subscriber sub2 = n.subscribe("odom", 10, &globalHandle::odom_callback, &listener);

    ros::Subscriber sub3 = n.subscribe("move_base_simple/goal", 10, &globalHandle::goal_callback, &listener);


    ros::spin();

    return 0;
}








// void globalHandle::inflate( vector<int8_t>& grids, const int x, const int y, const vector<vector<int8_t>>& inflate_sample_in , const int radius  )
// {
//     // int x_start = x - radius;
//     // int x_end   = x + radius + 1 ;
//     // int y_start = y - radius;
//     // int y_end   = y + radius + 1 ;
//     // int width = radius*2+1;

//     int x_start = std::max( x - radius, 0 );
//     int x_end   = std::min( x + radius+ 1, map_info_.grid_width );
//     int y_start = std::max( y - radius, 0 );
//     int y_end   = std::min( y + radius+ 1, map_info_.grid_width );
//     int width = x_end - x_start  ;

//     // cout << "inflate: " << x_start << " : " << x_end  << "  " << y_start << " : " << y_end << endl;

//     int ycounter = 0;
//     for(auto arow : inflate_sample_in)
//     {
//         int yth = y_start+ycounter;
//         int start_index = yth*map_info_.grid_width +  x_start;
//         int xcounter =0;
//         while (xcounter < width)
//         {
//             if(arow[xcounter] != 0)
//             {
//                 grids[ start_index + xcounter ] = arow[xcounter];
                
//             }
//             xcounter ++;
//         }
//         ycounter ++ ;
//     }
    
// }



// void globalHandle::build_inflate_sample( std::vector< std::vector<int8_t> >& inflate_sample_target, const int radius )
// {
//     int step_size = 100/(radius+1);
//     int center = radius;

//     for(int i = 0; i<radius*2+1; i++)
//     {
//         std::vector<int8_t> a_row;
//         for(int j = 0; j<radius*2+1; j++)
//         {
//             float dx = i-center;
//             float dy = j-center;
//             if( dx*dx + dy*dy < radius*radius )
//             {
//                 a_row.push_back(100);
//                 cout << "1 ";
//             }
//             else
//             {
//                 a_row.push_back(0);
//                 cout << "0 ";
//             }
//         }
//         cout << endl;
//         inflate_sample_target.push_back(a_row);
//     }
// }


// float globalHandle::rectify( float a)
// {
//     float angle = a;
//     while (angle > 2*M_PI)
//     {
//         angle -= 2*M_PI;
//     }
//     while (angle < 0)
//     {
//         angle += 2*M_PI;
//     }
//     return angle;
// }







// void globalHandle::get_tf_laser_to_base(std::array<float, 3> &the_tf)
// {
//     tf::TransformListener listener;
//     tf::StampedTransform transform;
//     bool got_tf = false;

//     while (!got_tf)
//     {
//         try
//         {
//             listener.lookupTransform("/base_link", "/base_laser_link", ros::Time(0), transform);
//             got_tf = true;
//         }
//         catch (tf::TransformException &ex)
//         {
//             ROS_ERROR("%s", ex.what());
//             ros::Duration(1.0).sleep();
//             continue;
//         }
//     }

//     the_tf[0] = transform.getOrigin().x();
//     the_tf[1] = transform.getOrigin().y();
//     the_tf[2] = transform.getRotation().x();

//     // cout << the_tf[0] << " "  << the_tf[1] << " " << the_tf[2] << endl;
// }




