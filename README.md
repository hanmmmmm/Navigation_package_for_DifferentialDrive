# navigation_package_V1
This is my implementation of a complete 2D navigation package, including global planner, local planner, and motion controller.

Two simulators are used. 
- The first is stage_ros. It uses a bmp image to build the world, and creates a differential-drive robot with 2D lidar.

- The second is Gazebo with turtlebot. The map is turtlebot_house, and the robot is a differential-drive robot with 2D lidar.

For slam, I am using the standard Gmapping. 



### ROS messages being used:
- geometry_msgs:  Twist, PoseStamped
- sensor_msgs:  LaserScan
- nav_msgs:  OccupancyGrid, MapMetaData, Odometry, Path 
- tf/transform_listener



### C++ libraries used:
- ros/ros.h
- vector, array, queue, string
- atomic, thread
- iostream, chrono, stdexcept, iomanip
- set, map, unordered_map
- limits, algorithm, math.h


----
### Global goal and Global path

The line of green arrows is the path, from current location to the goal location.

This example is using A* algorithm to find the path. 

Each path-finding takes <1~20 ms. The more obstacles between robot and goal, the longer time it takes to find the valid path.

But this node is set to update the global path about 2~3 Hz even though it could do much faster, since global path doesn't need to be updated too frequently. 

Here I am dragging the robot in Stage simulator by hand.

<a id="search" href="https://github.com/hanmmmmm/navigation_package_V1/blob/main/gifs/nav_global_path.gif">
    <img src="https://github.com/hanmmmmm/navigation_package_V1/blob/main/gifs/nav_global_path.gif" alt="goal gif" title="set goal" width="750"/>
</a>



----
### Local goal and Local path

The green arrows are global path, and the red ones are local path.

This example is using Hybrid-A* to find the local path. The local goal is the last glocal path-point covered inside the local search area (The white square area around the robot base_link). 

<a id="search" href="https://github.com/hanmmmmm/navigation_package_V1/blob/main/gifs/nav_local_path.gif">
    <img src="https://github.com/hanmmmmm/navigation_package_V1/blob/main/gifs/nav_local_path.gif" alt="local gif" title="local" width="750"/>
</a>





----
### Working demo

Several new goals are added while the robot is moving. The paths are updated to guide the robot. 

The controller module use <i>odometry</i> and local_path to generate the disired linear & angular velocity for the robot to follow the local_path. 

<a id="search" href="https://github.com/hanmmmmm/navigation_package_V1/blob/main/gifs/nav_demo_1.gif">
    <img src="https://github.com/hanmmmmm/navigation_package_V1/blob/main/gifs/nav_demo_1.gif" alt="local gif" title="local" width="750"/>
</a>


----
### RQT_graph

This is the workflow within the system. 

The nodes StageROS and GMapping are standard ROS packages. 

The rest 3 modules are developed by myself.

<a id="search" href="https://github.com/hanmmmmm/navigation_package_V1/blob/main/gifs/nav_rqt.gif">
    <img src="https://github.com/hanmmmmm/navigation_package_V1/blob/main/gifs/nav_rqt.gif" alt="local gif" title="local" width="950"/>
</a>




