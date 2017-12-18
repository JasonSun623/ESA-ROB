ESA-ROS Assignment 6
--------------------

Minh-Triet Diep, Lars Jaeqx, Luuk van Rossum, Hubert Heijnen

# Creating a map - Gmapping / SLAM

When creating a map we start the navigation stack in mapping mode using the following command:

``` bash
roslaunch faw map_create.launch
```

This launches the navigation stack with `Gmapping-Slam` and enables the user to manually drive around the robot using a joystick. It is recommended to launch rviz using the command below (in a new terminal from the workspace). Please note that you need to source the setup bash file before running rviz. Otherwise the robot model is not correctly loaded. In rviz add the map and robotmodel to view the mapping progress clearly.

``` bash
source devel/setup.bash &&
rosrun rviz rviz
```

When the map looks good in rviz it's time to save it. To do so use the following commands (from the workspace):

``` bash
source devel/setup.bash &&
roscd faw/ &&
rosrun map_server map_saver -f <mapname>
```

## Joystick

Currently, the [navigation stack starts the nodes to get a controller working](https://github.com/minhtrietdiep/ESA-PROJ/blob/master/src/youbot/faw/templates/youbot_control/gamepad.template). This controller, a Playstation 4 controller, is connected to the controlling laptop with Bluetooth.

The [`src/youbot/faw/config/ps3.config.yaml`](https://github.com/minhtrietdiep/ESA-PROJ/blob/master/src/youbot/faw/config/ps3.config.yaml) file is used for reading the controller parameters, and the nodes `youbot_joy_node` and `teleop_twist_joy` translate controller output to `cmd_vel` commands. This controller is used during mapping and to move the robot directly.

### Joystick Controls

The left stick controls the movement, with the forward speed in the Y-axis of the stick and the rotation with the X-axis of the stick. The right stick is reserved for controlling the arm, but this might be subject to change.

On the PS4 controller:
* X moves the robot quickly. __DO NOT__ use this mode while navigating or anything related to accurate movement.
* O moves the robot slowly. This speed is suitable for the sensors and the robot tracks its position.

The slow movement speed should always be used, unless navigating is not necessary. If the robot has been moved at high speed, the navigation stack needs to be restarted with the robot in its home position, for it to be able to accurately locate itself again.

#  Moving the robot - AMCL

The robot can be told to autonomously move using Rviz, we start the navigation stack using the following launch file.
 
```sh
roslaunch faw costmap_test.launch
```

This will start the ros navigation stack and all of our custom nodes: the nodes for getting data from the Lidars, the youbot driver, the map server, the move base node and then our custom nodes, a emergency stop, a node for controlling the robot with a controller and a node for creating an extra costmap layer that creates zones for no-go zones that the robot can’t drive through.

We then start Rviz so we can give Nav 2D goals to the robot base:
 
```sh
rosrun rviz rviz
```

Rviz sends this position to `/move_base_simple/goal` and the robot wil start moving.

# Creating navigator node

## Code explanation

Using the [example](http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals) we’ve build a simple client that sends a couple goals to move_base. We chose the points by driving the robot around the map, and saved the coordinates and rotations with our GUI. It sends a goal, and if the robot succeeds in reaching this goal, it will send the next goal. This continues until all goals are reached.

## Running instructions

First we have to start the navigation stack as described above.

As with the previous assignments, the steps are similar to get the program started:

```sh
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch assignment6 assigment6.launch
```

# Tests and observations

For the project we did some tests on the robot navigation precision and drift over time. The results of this and the various parameters we changed can be found in the following document:

[Testing movement precision](https://github.com/minhtrietdiep/ESA-PROJ/wiki/NavPrecisionTest.pdf)

For ESA-ROB, we also recorded the mapping and navigation test with our current navigation stack for the project. 

[YouTube screencapture](https://youtu.be/D0OkfzULC9E)  
[YouTube real-life](https://youtu.be/NuBk4oz7AMQ)

Now we created the navigator node and tested the results.

[YouTube navigator node](https://youtu.be/nKLArBWd_xw)

