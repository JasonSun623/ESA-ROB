ESA-ROS Assignment 6
--------------------

Minh-Triet Diep, Lars Jaeqx

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






# Code explanation

Hoi

# Running instructions

Hallo wereld wereld, de wereld is van mij. 
Er is ruimte zat, dus kom er lekker bij.
Hallo wereld wereld, de wereld is van jou. 
Er zijn wel duizend kleuren, veel meer dan rood-wit-blauw.
En wat ik doe doe doe, doe ik samen met jou,
en waar ik ga ga ga, ga ik samen met jou.
Hallo wereld, hééééééé!
Ja jij, je bent van ons allemaal.

# Tests and observations
