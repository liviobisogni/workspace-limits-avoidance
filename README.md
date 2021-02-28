# __Workspace Limits Avoidance__

### _Author_: Livio Bisogni
###### __&copy; 2021 Turtley & Turtles Ltd.__
___
Turtles go krazy!

## Prerequisites

* [ROS](http://wiki.ros.org/ROS/Installation) - An open-source, meta-operating system for your robots. Repository tested only under ROS Kinetic, though.

## How to compile
1. Move this folder (`workspace_limits_avoidance`) in `~/catkin_ws/src` (or wherever thy ROS workspace is).
2. Launch a terminal window and navigate to the aforementioned ROS workspace, e.g.,

	```
	$ cd ~/catkin_ws/
	```
3. Build the package:

	```
	$ catkin_make
	```

## How to execute
Open the terminal and type:

```
$ roslaunch workspace_limits_avoidance workspace_limits_avoidance.launch
```

## How to use1. The turtle pseudo-randomly moves within the bounds, so that walls are avoided.2. Once a wall is almost reached the turtle stops.3. Then, it goes back a bit and pseudo-randomly rotates.4. Finally, it moves forward again, and so forth.5. The turtle goes on indefinitely. Meanwhile, various types of information are printed on the terminal. Press `ESC` to exit the program anytime.

![](img/w1.png)

![](img/w2.png)

![](img/w3.png)