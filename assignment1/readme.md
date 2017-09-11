# ESA-ROB

Assignment 1. This will create a node that draws a triangle using the turtlesim simulator.  

This implements the following requirements::
* The node should subscribe to a topic called /cmd. On this topic it should be able to receive Triangle messages, defined as follows:  
`float32 sideLength`  
`bool cw`
* When receiving a Triangle message the node makes the turtle draw one red triangle with the given side length. The firts side of the triangle must be horizontal. The bool cw should make the turtle drive clockwise (cw == true) or counterclockwise (cw == false).
* If sideLength is greater than the distance to the wall the turtle is heading to, adjust sideLength to the maximum possible value without hitting the wall
* No lines except the triangle(s) should be visible in the turtlesim window, i.e. you should switch the turtle’s pen on/off (using Service calls to turtlesim).
* The package should contain a launch file `assignment1.launch` that starts both turtlesim_node and draw_triangle_node.

## Code explaniation

## Running Assignment

* `cd ~/catkin_ws`
* `catkin_make`
* `catkin_make install`
* `roscore`
* `roslaunch assignment1 assignment1.launch`
* `rostopic pub -1 /cmd assignment1/Triangle -- 1.2 true`

## ROS comuptation graph
[ROS Computation Graph](rosgraph.svg)