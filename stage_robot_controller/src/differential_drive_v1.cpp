
#include "../include/stage_robot_controller/differential_drive_v1.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/Twist.h"

#include <thread>

Diff_drive_v1_class::Diff_drive_v1_class(const ros::NodeHandle nh_in): pathPoint(), nh_{nh_in}
{
    cout << "Diff_drive_v1_class  init " << endl;

    cout << "setup variables" << endl;
    
    // FLAG_atom_local_goal_available.store(false);
    FLAG_atom_local_path_in_use.store(false);
    // FLAG_atom_global_path_in_use.store(false);
    // FLAG_atom_local_map_in_use.store(false);
    FLAG_atom_odom_in_use.store(false);
    
    pose_pred_duration_ = 1.0;

    map_size_ = 2.0;

    gene_cmdvel_time_interval_ = 0.1 ;


    cout << "sub and pub" << endl;

    // local_map_puber_ = nh_.advertise<nav_msgs::OccupancyGrid>("local_map", 10);
    // local_path_puber_ = nh_.advertise<nav_msgs::Path>("local_path", 10);
    cmd_vel_puber_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    odom_suber_ = nh_.subscribe("odom", 10, &Diff_drive_v1_class::odom_callback, this);
    local_path_suber_ = nh_.subscribe( "local_path", 10, &Diff_drive_v1_class::local_path_callback, this );

    periodic_cmdvel_ = nh_.createTimer( ros::Duration(gene_cmdvel_time_interval_), &Diff_drive_v1_class::gene_cmdvel, this );



}

Diff_drive_v1_class::~Diff_drive_v1_class()
{
}



void Diff_drive_v1_class::gene_cmdvel( const ros::TimerEvent &event )
{
    cout << "gene_cmdvel " << endl;
    float v = 0.3;
    float w = 0.3;

    // cout << "xxxxx " << endl;
    
    // while( FLAG_atom_local_path_in_use.load() )
    // {
    //     std::this_thread::yield();
    // }
    // FLAG_atom_local_path_in_use.store(true);

    // for(auto pppp : path_)
    // {
    //     cout << pppp.x << " " << pppp.y << endl;
    // }

    float px, py;
    for(int i =0; i<path_.size(); i++)
    {
        px = path_[i].x;
        py = path_[i].y;
        cout << path_[i].x << " " << path_[i].y << endl;
        if( i == 3)
        {
            break;
        }
    }

    // FLAG_atom_local_path_in_use.store(false);

    cout << "px " << px << "  py " << py << endl;

    float d = sqrt( px*px + py*py );
    float sin_alpha = py/d;

    v = 0.5 * px;
    w = 2 * v * sin_alpha / d;

    cmdvel_msg_.linear.x = v;
    cmdvel_msg_.angular.z = w;

    cmd_vel_puber_.publish(cmdvel_msg_);

}



void Diff_drive_v1_class::local_path_callback(const nav_msgs::Path::ConstPtr &msg)
{
    while( FLAG_atom_local_path_in_use.load() )
    {
        std::this_thread::yield();
    }
    FLAG_atom_local_path_in_use.store(true);

    cout << "local_path_callback using  local_path" << endl;

    path_.clear();
    for(auto p : msg->poses)
    {
        pathPoint pp;
        pp.x = p.pose.position.x;
        pp.y = p.pose.position.y;
        tf::Quaternion q(
            p.pose.orientation.x,
            p.pose.orientation.y,
            p.pose.orientation.z,
            p.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        pp.heading = rectify(yaw);

        path_.push_back(pp);

        // cout << "px " << pp.x << "  py " << pp.y << endl;
    }

    FLAG_atom_local_path_in_use.store(false);
    cout << "local_path_callback released  local_path" << endl;
}


void Diff_drive_v1_class::odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
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

    robot_pose_.yaw_rad = yaw;
    robot_pose_.x_meter = msg->pose.pose.position.x;
    robot_pose_.y_meter = msg->pose.pose.position.y;
    float v = msg->twist.twist.linear.x;
    float w = msg->twist.twist.angular.z;

    robot_pose_.x_pred = robot_pose_.x_meter + v*cos(yaw)*pose_pred_duration_ ;
    robot_pose_.y_pred = robot_pose_.y_meter + v*sin(yaw)*pose_pred_duration_ ;
    robot_pose_.yaw_pred = rectify( robot_pose_.yaw_rad + w * pose_pred_duration_ ) ;

    FLAG_atom_odom_in_use.store(false);

    if(v != 0.0 || w != 0.0)
    {
        cout << std::fixed << std::setprecision(3)  << "curr pose: " << robot_pose_.x_meter << " "  << robot_pose_.y_meter << " " << robot_pose_.yaw_rad << " ; pred pose " ;
        cout << std::fixed << std::setprecision(3)  << robot_pose_.x_pred << " "  << robot_pose_.y_pred << " " << robot_pose_.yaw_pred << endl;
    }

}




float Diff_drive_v1_class::rectify( float a)
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






int main(int argc, char **argv)
{
    ros::init(argc, argv, "drive_control_node");
    ros::NodeHandle n;
    
    Diff_drive_v1_class cmdvel( n );
    
    ros::AsyncSpinner s(2);
    s.start();

    ros::waitForShutdown();

    return 0;
}

