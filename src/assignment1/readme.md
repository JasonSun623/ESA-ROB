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

## Code explanation

Firstly we implemented `Triangle.msg`, which is simple as it only existed of two lines denoting the type and name:

```
float32 sideLength
bool cw
```

The main code is in `draw_triangle_node.cpp`, where the main body publishes a `cmd_vel` topic of the `Twist` message type. It subscribes to `cmd` and `/turtle1/pose` and has callbacks to handle messages from these topics. The turtle also has a `turtle1/set_pen` service we use to raise and lower the pen.

The pose callback simply updates the `turtlesim_pose` global. The cmd callback responds to a `Triangle` message and makes the turtle draw a triangle with the specified parameters when called.

After getting the value from the parameters in the `Triangle` message the function calculates if the turtle will hit the wall. If it does, the distance is set to a value slightly less than the distance to the wall. This distance is taken from tests (reading the log messages when the turtle hits the wall) and the information from the turtlesim documentation that the window is fixed-size. It then performs a hard-coded set of moves and rotates to make a triangle.

The `move` function prepares a `Twist` message with just speed, and publishes this. To measure distance, timing was used. Knowing the speed and the time from start to the current running time, the message is sent until the calculated planned distance is reached. After this, the speed is zeroed.

The `rotate` function uses a similar approach, but with angular speeds to reach the desired rotation. The units are in radians, so we converted our inputs from degrees to radian before calling this, for convenience reasons.

We noticed the turtle having a slight inaccuracy while rotating and moving, but the rotation inaccuracy is very noticable after subsequent runs, so we also made it rotate until a sufficiently small `theta` from the `turtlesim_pose` was reached. After all this, the pen is lifted up again.



## Running Assignment

* `cd ~/catkin_ws`
* `catkin_make`
* `catkin_make install`
* `source devel/setup.bash`
* `roscore`
* `roslaunch assignment1 assignment1.launch`
* `rostopic pub -1 /cmd assignment1/Triangle -- 1.2 true`

[Demo](https://youtu.be/keSguKYp9J0)

## ROS comuptation graph
![ROS Computation Graph](rosgraph.svg)

