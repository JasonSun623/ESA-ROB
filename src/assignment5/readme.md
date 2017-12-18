ESA-ROS Assignment 5
--------------------

Minh-Triet Diep, Lars Jaeqx

# Code explanation

Firstly we implemented `Triangle.msg`, which is simple as it only existed of two lines denoting the type and name:

```
float32 sideLength
bool cw
```

The main code is in `move_triangle.cpp`, it subscribes to `cmd` and has a callback to handle messages from this topic. In this callback we retrieve the distance and the rotation direction. Using a `SimpleActionClient` we send 6 actions the robot has to perform, 3 moves and 3 turns.

Using `sendGoal(goal, &doneCb, &activeCb, &feedbackCb)` we print the feedback in the callback functions.

# Running instructions  

As with the previous assignments, the steps are similar to get the program started:

```sh
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch assignment5 assigment5.launch
```


# Tests and Observations

During testing we saw the robot behaving like we expected, executing the 6 tasks of driving and rotating in order. We did notice the final rotation going too far. We checked this with other teams, who also had the same problem. 

We also wanted to print feedback with callbacks, but we didn't see this in the terminal window. We tried to figure out why this was like this, but didn't manage to find out to get it to work. The arguments are done in a similar way to examples and we checked the API, but everything seemed right.

[YouTube result](https://youtu.be/dDmTUc4Fod8)

