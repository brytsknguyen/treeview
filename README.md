# treeview: package to view the trees
<!-- via Continuous-time Optimization -->

# Prerequisite

The software was developed on the following dependancies
1. [Ubuntu 20.04](https://releases.ubuntu.com/20.04/)
2. [ROS Noetic](http://wiki.ros.org/noetic/Installation)
3. [PCL]

# Installation
Please install all dependencies first. Afterwards, create a ros workspace, clone the package to the workspace, and build by `catkin build` or `catkin_make`, for e.g.:

```
mkdir catkin_ws/src
cd catkin_ws/src
git clone https://github.com/brytsknguyen/treeview
cd ..; catkin build
```

# Launch

After Installation, add the path of the rosbags to the launch file `launch/run.launch` and then launch it to view the data.

