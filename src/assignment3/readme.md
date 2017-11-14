ESA-ROS Assignment 3
--------------------

Minh-Triet Diep, Lars Jaeqx

# Code explanation

## Global planner  
For this assignment we have to make two global planners. One has to follow a rectangular path and the other has to make a triangular path. We have to advertise this path to the `/plan` topic.  
  
The `/plan` contains a vector with geometry_msgs::PoseStamped of the waypoints we have to follow. In `makePath` we fill the vector with the waypoints.

## Local planner (follow the carrot)  
The local planner was a bit more complex. First we tried to think about how to nicely be able to follow a virtual carrot. Drawing a circle with a radius of the look-ahead distance, and finding the intersections on the path gave us two points. One of these points, the one closest to the "next goal" is the one we want to follow.

We tried to calculate this by ourselves instead of using some library, for a nostalgic throwback to high school. Sadly we made a series of mistakes, so we spent a whole day on calculating these intersections. The errors we made are sloppy, forgetting to square several terms and just miscalculations.

After that worked, we also had trouble getting the robot to follow the calculated point. 



# Running instructions  

As with the previous assignment, the steps are similar to get the program started:

```sh
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch assignment3 assigment3.launch
```

To publish goals:

```sh
?????
```

# Tests and Observations  

Nee

# Graph 
Nee stop
