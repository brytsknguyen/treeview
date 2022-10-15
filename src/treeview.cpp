//
// Created by Thien-Minh Nguyen on 15/12/20.
//

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/pcl_base.h>
#include <pcl/impl/pcl_base.hpp>

#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PointStamped.h"
#include "livox_ros_driver/CustomMsg.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <Eigen/Dense>

using namespace std;
using namespace pcl;
using namespace Eigen;
using namespace message_filters;

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
    
    ros::Publisher ousterCloudPub;

    message_filters::Subscriber<livox_ros_driver::CustomMsg> livoxCloudSub;
    message_filters::Subscriber<geometry_msgs::PointStamped> djiPosSub;
    message_filters::Subscriber<sensor_msgs::Imu> djiImuSub;

    typedef sync_policies::ApproximateTime<livox_ros_driver::CustomMsg, geometry_msgs::PointStamped, sensor_msgs::Imu> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync;

public:
    // Destructor
    ~AirForestViz() {}

    AirForestViz(ros::NodeHandlePtr &nh_ptr_)
        : nh_ptr(nh_ptr_),
          sync(MySyncPolicy(10), livoxCloudSub, djiPosSub, djiImuSub)
    {
        NUM_THREAD = omp_get_max_threads();

        // Coefficient to convert the intensity from livox to ouster
        nh_ptr->param("intensityConvCoef", intensityConvCoef, 1.0);

        // Subscribe to the livox topic
        // livoxCloudSub  = nh_ptr->subscribe<livox_ros_driver::CustomMsg>("/livox/lidar", 50, &AirForestViz::livoxCloudHandler, this, ros::TransportHints().tcpNoDelay());
        ousterCloudPub = nh_ptr->advertise<sensor_msgs::PointCloud2>("/livox/lidar_ouster", 50);

        livoxCloudSub.subscribe(*(nh_ptr), "/livox/lidar", 1);
        djiPosSub.subscribe(*(nh_ptr), "/dji_osdk_ros/local_position", 1);
        djiImuSub.subscribe(*(nh_ptr), "/dji_osdk_ros/imu", 1);

        sync.registerCallback(boost::bind(&AirForestViz::djiCloudImuPosHandler, this, _1, _2, _3));
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

    void djiCloudImuPosHandler(const livox_ros_driver::CustomMsg::ConstPtr &CloudMsg,
                               const geometry_msgs::PointStamped::ConstPtr &PosMsg,
                               const sensor_msgs::Imu::ConstPtr &ImuMsg)
    {

        printf("Cloud Time: %.3f. Pos time: %.3f. Imu time: %.3f. Time diff: %6.3f\n",
                CloudMsg->header.stamp.toSec(),
                PosMsg->header.stamp.toSec(), ImuMsg->header.stamp.toSec(),
                PosMsg->header.stamp.toSec() - ImuMsg->header.stamp.toSec());

        int cloudsize = CloudMsg->points.size();

        CloudOuster laserCloudOuster;
        laserCloudOuster.points.resize(cloudsize);
        laserCloudOuster.is_dense = true;

        #pragma omp parallel for num_threads(NUM_THREAD)
        for (size_t i = 0; i < cloudsize; i++)
        {
            auto &src = CloudMsg->points[i];
            auto &dst = laserCloudOuster.points[i];
            dst.x = src.x;
            dst.y = src.y;
            dst.z = src.z;
            dst.intensity = src.reflectivity * intensityConvCoef;
            dst.ring = src.line;
            dst.t = src.offset_time;
            dst.range = sqrt(src.x * src.x + src.y * src.y + src.z * src.z);
        }

        Quaterniond q(ImuMsg->orientation.w, ImuMsg->orientation.x, ImuMsg->orientation.y, ImuMsg->orientation.z);
        Vector3d p(PosMsg->point.x, PosMsg->point.y, PosMsg->point.z);

        pcl::transformPointCloud(laserCloudOuster, laserCloudOuster, p, q);

        publishCloud(ousterCloudPub, laserCloudOuster, CloudMsg->header.stamp, "world");
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