
# Prerequisites
1. ROS
2. ros-$DISTRO-navigation

# Prerequisites for turtlebot navigation demo

Install dependent ros packages for turtle bot
    http://emanual.robotis.com/docs/en/platform/turtlebot3/pc_setup/#install-dependent-ros-packages

# Installation

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
cd src/
git clone https://github.com/Mayavan/ros_global_planning_plugins.git
cd ..
catkin_make
source devel/setup.bash
```

# To run demo
## Available Planners
* bfs_planner/BFSPlanner

```
roslaunch demo demo.launch global_planner:=planner_name
```
