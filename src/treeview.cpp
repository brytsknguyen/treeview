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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PointStamped.h"
#include "livox_ros_driver/CustomMsg.h"

using namespace std;
using namespace pcl;

struct PointOuster
{
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t  ring;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(PointOuster,
                                 (float, x, x) (float, y, y) (float, z, z)
                                 (float, intensity, intensity)
                                 (uint32_t, t, t)
                                 (uint16_t, reflectivity, reflectivity)
                                 (uint8_t,  ring, ring)
                                 (uint32_t, range, range))

typedef pcl::PointCloud<PointOuster> CloudOuster;
typedef pcl::PointCloud<PointOuster>::Ptr CloudOusterPtr;

class AirForestViz
{
private:
    // Node handler
    ros::NodeHandlePtr nh_ptr;

    int NUM_THREAD;
    double intensityConvCoef = -1;
    ros::Subscriber livoxCloudSub;
    ros::Publisher  ousterCloudPub;

    ros::Subscriber djiImuSub;
    ros::Subscriber djiPosSub;

public:
    // Destructor
    ~AirForestViz() {}

    AirForestViz(ros::NodeHandlePtr &nh_ptr_) : nh_ptr(nh_ptr_)
    {
        NUM_THREAD = omp_get_max_threads();

        // Coefficient to convert the intensity from livox to ouster
        nh_ptr->param("intensityConvCoef", intensityConvCoef, 1.0);

        // Subscribe to the livox topic
        livoxCloudSub  = nh_ptr->subscribe<livox_ros_driver::CustomMsg>("/livox/lidar", 50, &AirForestViz::livoxCloudHandler, this, ros::TransportHints().tcpNoDelay());
        ousterCloudPub = nh_ptr->advertise<sensor_msgs::PointCloud2>("/livox/lidar_ouster", 50);

        // Subscribe to the position topic
        djiImuSub = nh_ptr->subscribe("/dji_osdk_ros/imu", 100, &AirForestViz::djiImuHandler, this);
        djiPosSub = nh_ptr->subscribe("/dji_osdk_ros/local_position", 100, &AirForestViz::djiPosHandler, this);
    }

    void livoxCloudHandler(const livox_ros_driver::CustomMsg::ConstPtr &msgIn)
    {
        int cloudsize = msgIn->points.size();

        CloudOuster laserCloudOuster;
        laserCloudOuster.points.resize(cloudsize);
        laserCloudOuster.is_dense = true;

        #pragma omp parallel for num_threads(NUM_THREAD)
        for (size_t i = 0; i < cloudsize; i++)
        {
            auto &src = msgIn->points[i];
            auto &dst = laserCloudOuster.points[i];
            dst.x = src.x;
            dst.y = src.y;
            dst.z = src.z;
            dst.intensity = src.reflectivity * intensityConvCoef;
            dst.ring = src.line;
            dst.t = src.offset_time;
            dst.range = sqrt(src.x * src.x + src.y * src.y + src.z * src.z);
        }

        publishCloud(ousterCloudPub, laserCloudOuster, msgIn->header.stamp, msgIn->header.frame_id);
    }

    void publishCloud(ros::Publisher &thisPub, pcl::PointCloud<PointOuster> &thisCloud, ros::Time thisStamp, std::string thisFrame)
    {
        sensor_msgs::PointCloud2 tempCloud;
        pcl::toROSMsg(thisCloud, tempCloud);
        tempCloud.header.stamp = thisStamp;
        tempCloud.header.frame_id = thisFrame;
        thisPub.publish(tempCloud);
    }

    void djiImuHandler(const sensor_msgs::Imu::ConstPtr msg)
    {

    }

    void djiPosHandler(const geometry_msgs::PointStamped::ConstPtr msg)
    {

    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "livox_to_ouster");
    ros::NodeHandle nh("~");
    ros::NodeHandlePtr nh_ptr = boost::make_shared<ros::NodeHandle>(nh);

    ROS_INFO("----> Livox to Ouster started");

    AirForestViz L2O(nh_ptr);

    ros::MultiThreadedSpinner spinner(0);
    spinner.spin();

    return 0;
}