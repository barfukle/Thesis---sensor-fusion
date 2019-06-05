#include <ros/ros.h>
#include <ros/console.h>
#include <pcl_ros/publisher.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>


ros::Publisher pub;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Input cloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*input, cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>(cloud));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

  // Sparse Filtering
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(temp_cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_filtered);


  // Publishes the data
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_filtered, output);
    pub.publish (output);

}



int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "lidar_sparse");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("solo_voxel_lidar", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("lidar_sparse_output", 1);

  // Spin
  ros::spin ();
}