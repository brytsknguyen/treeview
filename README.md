# airforest_viz: Surfel-based Lidar-Inertial Odometry and Mapping
<!-- via Continuous-time Optimization -->

# Prerequisite

The software was developed on the following dependancies
1. [Ubuntu 20.04](https://releases.ubuntu.com/20.04/)
2. [ROS Noetic](http://wiki.ros.org/noetic/Installation)
3. [Ceres 2.1.0](http://ceres-solver.org/installation.html)
4. [UFOMap (devel_surfel)](https://github.com/brytsknguyen/ufomap/tree/devel_surfel)

# Installation
Please install all dependencies first. Afterwards, create a ros workspace, clone the package to the workspace, and build by `catkin build` or `catkin_make`, for e.g.:

```
mkdir catkin_ws/src
cd catkin_ws/src
git clone https://github.com/brytsknguyen/airforest_viz
cd ..; catkin build
```

