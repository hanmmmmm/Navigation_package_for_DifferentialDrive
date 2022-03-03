# navigation_package_V1
This is my implementation of a complete 2D navigation package, including global planner, local planner, and motion controller.


ROS messages being used:
- geometry_msgs/Twist.h
- geometry_msgs/PoseStamped.h
- tf/transform_listener.h
- sensor_msgs/LaserScan.h
- nav_msgs/OccupancyGrid.h
- nav_msgs/MapMetaData.h
- nav_msgs/Odometry.h
- nav_msgs/Path.h


----
### Global goal and Global path

The line of green arrows is the path, from current location to the goal location.

This example is using A* algorithm to find the path. 

Each path-finding takes <1~20 ms. The more obstacles between robot and goal, the longer time it takes to find the valid path.

This node is set to update the global path about 2~3 Hz even though it could do much faster, since global path doesn't need to be updated too frequently. 

<a id="search" href="https://github.com/hanmmmmm/navigation_package_V1/blob/main/gifs/nav_global_path.gif">
    <img src="https://github.com/hanmmmmm/navigation_package_V1/blob/main/gifs/nav_global_path.gif" alt="goal gif" title="set goal" width="750"/>
</a>



----
### Local goal and Local path

The green arrows are global path, and the red ones are local path.


<a id="search" href="https://github.com/hanmmmmm/navigation_package_V1/blob/main/gifs/nav_local_path.gif">
    <img src="https://github.com/hanmmmmm/navigation_package_V1/blob/main/gifs/nav_local_path.gif" alt="local gif" title="local" width="750"/>
</a>





----
### Working demo

Several new goals are added while the robot is moving. The paths are updated to guide the robot. 


<a id="search" href="https://github.com/hanmmmmm/navigation_package_V1/blob/main/gifs/nav_demo_1.gif">
    <img src="https://github.com/hanmmmmm/navigation_package_V1/blob/main/gifs/nav_demo_1.gif" alt="local gif" title="local" width="750"/>
</a>


----
### RQT_graph

<a id="search" href="https://github.com/hanmmmmm/navigation_package_V1/blob/main/gifs/nav_rqt.gif">
    <img src="https://github.com/hanmmmmm/navigation_package_V1/blob/main/gifs/nav_rqt.gif" alt="local gif" title="local" width="950"/>
</a>




