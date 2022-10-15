//
// Created by Thien-Minh Nguyen on 15/12/20.
//

#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <condition_variable>
#include <deque>
#include <thread>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "livox_ros_driver/CustomMsg.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "utility.h"

using namespace std;
using namespace pcl;
using namespace Eigen;
using namespace message_filters;

class AirForestViz
{
private:
    // Node handler
    ros::NodeHandlePtr nh_ptr;

    int NUM_THREAD;
    double intensityConvCoef = -1;
    ros::Subscriber livoxCloudSub;
    ros::Publisher  ousterCloudPub;
    ros::Publisher  posePub;
    ros::Publisher  ousterCloudTfPub;

    message_filters::Subscriber<geometry_msgs::PointStamped> djiPosSub;
    message_filters::Subscriber<sensor_msgs::Imu> djiImuSub;

    typedef sync_policies::ApproximateTime<geometry_msgs::PointStamped, sensor_msgs::Imu> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync;

    typedef geometry_msgs::PoseStamped RosPose;
    deque<RosPose> pose_buf;
    mutex pose_buf_mtx;

    deque<pair<double, CloudOusterPtr>> cloud_buf;
    mutex cloud_buf_mtx;

    Matrix3d R_B_L;

public:
    // Destructor
    ~AirForestViz() {}

    AirForestViz(ros::NodeHandlePtr &nh_ptr_)
        : nh_ptr(nh_ptr_),
          sync(MySyncPolicy(10), djiPosSub, djiImuSub)
    {
        NUM_THREAD = omp_get_max_threads();

        // Coefficient to convert the intensity from livox to ouster
        nh_ptr->param("intensityConvCoef", intensityConvCoef, 1.0);

        // Subscribe to the livox topic
        livoxCloudSub  = nh_ptr->subscribe<livox_ros_driver::CustomMsg>("/livox/lidar", 50, &AirForestViz::livoxCloudHandler, this, ros::TransportHints().tcpNoDelay());
        ousterCloudPub = nh_ptr->advertise<sensor_msgs::PointCloud2>("/livox/lidar_ouster", 50);

        djiPosSub.subscribe(*(nh_ptr), "/dji_osdk_ros/local_position", 1);
        djiImuSub.subscribe(*(nh_ptr), "/dji_osdk_ros/imu", 1);

        sync.registerCallback(boost::bind(&AirForestViz::djiImuPosHandler, this, _1, _2));

        posePub = nh_ptr->advertise<RosPose>("/dji/uav_pose", 50);
        ousterCloudTfPub = nh_ptr->advertise<sensor_msgs::PointCloud2>("/livox/lidar_ouster_tf", 50);

        R_B_L << 0,  0, -1,
                 0, -1,  0,
                -1,  0,  0;
    }

    void ProcessData()
    {
        while(true)
        {
            if (cloud_buf.empty() || pose_buf.empty())
            {
                this_thread::sleep_for(chrono::milliseconds(25));
                continue;
            }
            
            if (CloudEndTime(cloud_buf.front()) > pose_buf.back().header.stamp.toSec())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(25));
                continue;
            }
            
            if (CloudStartTime(cloud_buf.front()) < pose_buf.front().header.stamp.toSec())
            {
                cloud_buf_mtx.lock();
                cloud_buf.pop_front();
                cloud_buf_mtx.unlock();
                continue;
            }

            // Extract the cloud and poses
            double cloudStartTime = CloudStartTime(cloud_buf.front());
            double cloudEndTime   = CloudEndTime(cloud_buf.front());
            CloudOusterPtr cloud  = cloud_buf.front().second;

            cloud_buf_mtx.lock();
            cloud_buf.pop_front();
            cloud_buf_mtx.unlock();

            // Extract the pose
            deque<RosPose> poses;
            while(!pose_buf.empty())
            {
                RosPose pose = pose_buf.front();

                if(pose_buf.front().header.stamp.toSec() < cloudStartTime)
                {
                    pose_buf_mtx.lock();
                    pose_buf.pop_front();
                    pose_buf_mtx.unlock();
                    continue;
                }

                if (cloudStartTime < pose.header.stamp.toSec() && pose.header.stamp.toSec() < cloudEndTime)
                {
                    poses.push_back(pose);

                    pose_buf_mtx.lock();
                    pose_buf.pop_front();
                    pose_buf_mtx.unlock();
                    continue;
                }
                else
                    break;
            }

            printf("Cloud %.3f, %.3f. Poses: %d. %.3f->%.3f\n", cloudStartTime, cloudEndTime, poses.size(), poses.front().header.stamp.toSec(), poses.back().header.stamp.toSec() );

            ROS_ASSERT(!poses.empty());

            Vector3d p(poses.front().pose.position.x, poses.front().pose.position.y, poses.front().pose.position.z);
            Quaternd q = Quaternd(poses.front().pose.orientation.w, poses.front().pose.orientation.y,
                                  poses.front().pose.orientation.y, poses.front().pose.orientation.z)*Quaternd(R_B_L);

            CloudOusterPtr cloudtf(new CloudOuster());
            pcl::transformPointCloud(*cloud, *cloudtf, p, q);

            publishCloud(ousterCloudTfPub, *cloudtf, ros::Time(cloudStartTime), "local");
        }
    }

    double CloudEndTime(pair<double, CloudOusterPtr> &cloudStamped)
    {
        return cloud_buf.front().first + cloud_buf.front().second->points.back().t/1.0e9;
    }

    double CloudStartTime(pair<double, CloudOusterPtr> &cloudStamped)
    {
        return cloud_buf.front().first;
    }

    void livoxCloudHandler(const livox_ros_driver::CustomMsg::ConstPtr &msgIn)
    {
        int cloudsize = msgIn->points.size();

        CloudOusterPtr laserCloudOuster(new CloudOuster());
        laserCloudOuster->points.resize(cloudsize);
        laserCloudOuster->is_dense = true;

        #pragma omp parallel for num_threads(NUM_THREAD)
        for (size_t i = 0; i < cloudsize; i++)
        {
            auto &src = msgIn->points[i];
            auto &dst = laserCloudOuster->points[i];
            dst.x = src.x;
            dst.y = src.y;
            dst.z = src.z;
            dst.intensity = src.reflectivity * intensityConvCoef;
            dst.ring = src.line;
            dst.t = src.offset_time;
            dst.range = sqrt(src.x * src.x + src.y * src.y + src.z * src.z);
        }

        publishCloud(ousterCloudPub, *laserCloudOuster, msgIn->header.stamp, msgIn->header.frame_id);

        cloud_buf_mtx.lock();
        cloud_buf.push_back(make_pair(msgIn->header.stamp.toSec(), laserCloudOuster));
        cloud_buf_mtx.unlock();
    }

    void publishCloud(ros::Publisher &thisPub, pcl::PointCloud<PointOuster> &thisCloud, ros::Time thisStamp, std::string thisFrame)
    {
        sensor_msgs::PointCloud2 tempCloud;
        pcl::toROSMsg(thisCloud, tempCloud);
        tempCloud.header.stamp = thisStamp;
        tempCloud.header.frame_id = thisFrame;
        thisPub.publish(tempCloud);
    }

    void djiImuPosHandler(const geometry_msgs::PointStamped::ConstPtr &PosMsg, const sensor_msgs::Imu::ConstPtr &ImuMsg)
    {
        printf("Pos time: %.3f. Imu time: %.3f. Time diff: %6.3f\n",
                PosMsg->header.stamp.toSec(), ImuMsg->header.stamp.toSec(),
                PosMsg->header.stamp.toSec() - ImuMsg->header.stamp.toSec());
        
        static geometry_msgs::PointStamped first_point = *PosMsg;

        geometry_msgs::PoseStamped pose;
        pose.header = PosMsg->header;
        pose.header.frame_id = "local";
        pose.pose.position = PosMsg->point;
        pose.pose.orientation = ImuMsg->orientation;

        pose.pose.position.x -= first_point.point.x;
        pose.pose.position.y -= first_point.point.y;
        pose.pose.position.z -= first_point.point.z;

        posePub.publish(pose);

        pose_buf_mtx.lock();
        pose_buf.push_back(pose);
        pose_buf_mtx.unlock();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "livox_to_ouster");
    ros::NodeHandle nh("~");
    ros::NodeHandlePtr nh_ptr = boost::make_shared<ros::NodeHandle>(nh);

    ROS_INFO("----> Livox to Ouster started");

    AirForestViz L2O(nh_ptr);

    std::thread process_data(&AirForestViz::ProcessData, &L2O);

    ros::MultiThreadedSpinner spinner(0);
    spinner.spin();

    return 0;
}