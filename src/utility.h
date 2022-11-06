/**
* This file is part of SLICT.
* 
* Copyright (C) 2020 Thien-Minh Nguyen <thienminh.nguyen at ntu dot edu dot sg>,
* School of EEE
* Nanyang Technological Univertsity, Singapore
* 
* For more information please see <https://britsknguyen.github.io>.
* or <https://github.com/britsknguyen/SLICT>.
* If you use this code, please cite the respective publications as
* listed on the above websites.
* 
* SLICT is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
* 
* SLICT is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
* 
* You should have received a copy of the GNU General Public License
* along with SLICT.  If not, see <http://www.gnu.org/licenses/>.
*/

//
// Created by Thien-Minh Nguyen on 15/12/20.
//

#pragma once

#ifndef _Util_LIDAR_ODOMETRY_H_
#define _Util_LIDAR_ODOMETRY_H_

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <random>
#include <mutex>

#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include "glob.h"

using namespace std;
using namespace Eigen;

#define KNRM  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"
#define RESET "\033[0m"

/* #region  Custom point type definition --------------------------------------------------------*/

struct PointOuster
{
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t  ring;
    // uint16_t ambient; // Available in NTU VIRAL and multicampus datasets
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(PointOuster,
                                 (float, x, x) (float, y, y) (float, z, z)
                                 (float, intensity, intensity)
                                 (uint32_t, t, t)
                                 (uint16_t, reflectivity, reflectivity)
                                 (uint8_t,  ring, ring)
                                //  (uint16_t, ambient, ambient)
                                 (uint32_t, range, range))

typedef pcl::PointCloud<PointOuster> CloudOuster;
typedef pcl::PointCloud<PointOuster>::Ptr CloudOusterPtr;

/* #endregion  Custom point type definition -----------------------------------------------------*/


// Shortened typedef matching character length of Vector3d and Matrix3d
typedef Eigen::Quaterniond Quaternd;
typedef Eigen::Quaterniond Quaternf;

#endif