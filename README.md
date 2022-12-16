# Home Service Robot
Home Service Robot is the final project in the Nanodegree Program.
A robot will move to a pick-up position for picking up a green cube box and simulate carrying the box to a drop-off position and drops it

## Description


## Prerequisites


## Build and Launch
### 1. Build
- Clone the project and initialize a catkin workspace
```
$ mkdir catkin_ws && cd catkin_ws
$ git clone https://github.com/luuvanluc/HomeServiceRobot.git src
$ cd src && catkin_init_workspace
```

- Navigate back to the `catkin_ws`folder and build the project
```
$ cd ..
$ catkin_make
```

### 2. Launch
#### Part 1: SLAM Testing
This task is to autonomously map the environment
- Launch `test_slam.sh` file
```
$ chmod +x ./src/test_slam.sh
$ ./src/test_slam.sh
```

- Search for the `xterminal` running the `keyboard_teleop` node, and start controlling the robot to map fully the environment
- [Optional] To save the map, open a terminal navigate to the directory where you want to save the map and then run the following command
```
$ rosrun map_server map_saver
```

#### Part 2: Localization and Navigation Testing
This task is to pick two different goals and test the robot's ability to reach them and orient itself with respect to them
- Launch `test_navigation.sh` file
```
$ chmod +x ./src/test_navigation.sh
$ ./src/test_navigation.sh
```
- Click the **2D Nav Goal** button in the toolbar of RViz, then click and drag on the map to send the goal to the robot. It will start moving to the goal
#### Part 3: Navigation Goal Node
This task is to write a node that will communicate with the ROS navigation stack and autonomously send successive goals for the robot to reach.

The C++ code of `pick_objects` node in `pick_objects/src/pick_objects.cpp`
- Launch `pick_objects.sh` file
```
$ chmod +x ./src/pick_objects.sh
$ ./src/pick_objects.sh
```

#### Part 4: Virtual Objects
This task of this project is to model a virtual object with markers in rviz. 
The virtual object is the one being picked and delivered by the robot, 
thus it should first appear in its pickup zone, and then in its drop off zone once the robot reaches it.

The C++ code of `visualization_marker` node in `add_markers/src/add_markers_test_virtual_objects.cpp`
- Launch `add_marker.sh` file
```
$ chmod +x ./src/add_marker.sh
$ ./src/add_marker.sh
```

#### Part 5: Home Service Robot
Now it’s time to simulate a full home service robot capable of navigating to pick up and deliver virtual objects
- Launch `home_service.sh` file
```
$ chmod +x ./src/home_service.sh
$ ./src/home_service.sh
```
