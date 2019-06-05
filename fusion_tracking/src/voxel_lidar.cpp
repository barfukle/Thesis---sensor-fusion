#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/common/angles.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>

ros::Publisher pub;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Converts to PCL
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Voxelization
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.2, 0.2, 0.2);
  sor.filter (cloud_filtered);

  // Conversions
  sensor_msgs::PointCloud2 output;
  pcl_conversions::moveFromPCL(cloud_filtered, output);
  pcl::PointCloud<pcl::PointXYZRGB> other_cloud;
  pcl::fromROSMsg(output, other_cloud);
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr conv_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  conv_cloud->swap(other_cloud);

  // Artificially makes all lidar points color white; important because otherwise depth colors are kept
  for (int i = 0; i < conv_cloud->points.size(); i++)
    {
      conv_cloud->points[i].r = 255;
      conv_cloud->points[i].g = 255;
      conv_cloud->points[i].b = 255;
    }
  
  // Publishes to ROS
  sensor_msgs::PointCloud2::Ptr output2 (new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*conv_cloud, *output2);
  output2->header.frame_id = cloud_msg->header.frame_id;
  pub.publish(output2);
  
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "fusion_lid_vox");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/synchronized_lidar", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("voxel_lidar_output", 1);

  // Spin
  ros::spin ();
}