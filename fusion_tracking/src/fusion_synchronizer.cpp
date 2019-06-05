#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace sensor_msgs;
using namespace message_filters;

ros::Publisher kinect_pub;
ros::Publisher lidar_pub;


void callback(const sensor_msgs::PointCloud2::ConstPtr& kinect, const sensor_msgs::PointCloud2::ConstPtr& lidar)
{

  kinect_pub.publish(kinect);

  lidar_pub.publish(lidar);

}


int main(int argc, char** argv)

{
  ros::init(argc, argv, "fusion_synchronizer");
  ros::NodeHandle nh;

  message_filters::Subscriber<sensor_msgs::PointCloud2> kinect_sub(nh, "/camera/depth_registered/points", 100);
  message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub(nh, "/velodyne_points", 100);

  kinect_pub = nh.advertise<sensor_msgs::PointCloud2>("/synchronized_kinect", 1000);

  lidar_pub = nh.advertise<sensor_msgs::PointCloud2>("/synchronized_lidar", 1000);

  typedef sync_policies::ApproximateTime<PointCloud2, PointCloud2> MySyncPolicy;

  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), kinect_sub, lidar_sub);

  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();
}


