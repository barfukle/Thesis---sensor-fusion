#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <pcl_ros/publisher.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

ros::Publisher pub;
ros::Publisher pub2;

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Gets Cluster Cloud from EC
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  pcl::fromROSMsg(*input, cloud);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cluster(new pcl::PointCloud<pcl::PointXYZRGB>(cloud));

  sensor_msgs::PointCloud2::Ptr output2 (new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*temp_cluster, *output2);
  output2->header.frame_id = input->header.frame_id;
  pub2.publish(output2);

  // XYZ Coordinate Extreme Extraction
  float min_x = temp_cluster->points[0].x;
  float max_x = temp_cluster->points[0].x;    
  float min_y = temp_cluster->points[0].y;    
  float max_y = temp_cluster->points[0].y;
  float min_z = temp_cluster->points[0].z;
  float max_z = temp_cluster->points[0].z;
  
  for (size_t i = 0; i < temp_cluster->points.size(); i++)
    {
        if (temp_cluster->points[i].x <= min_x)
          {
            min_x = temp_cluster->points[i].x;
          }
        else if (temp_cluster->points[i].y <= min_y)
          {
            min_y = temp_cluster->points[i].y;
          }
        else if (temp_cluster->points[i].z <= min_z)
          {
            min_z = temp_cluster->points[i].z;
          }
        else if (temp_cluster->points[i].x >= max_x)
          {
            max_x = temp_cluster->points[i].x;
          }
        else if (temp_cluster->points[i].y >= max_y)
          {
            max_y = temp_cluster->points[i].y;
          }
        else if (temp_cluster->points[i].z >= max_z)
          {
            max_z = temp_cluster->points[i].z;
            
          }
        
    }

  // Size Comparison  
  if ( max_z - min_z < 1 && max_x - min_x < 1 && max_y - min_y > 0.7) //real time = 1.0, 1.0
    {
        for (int i = 0; i < temp_cluster->points.size(); i++)
        {
            temp_cluster->points[i].r = 255;
            temp_cluster->points[i].g = 0;
            temp_cluster->points[i].b = 0;
        }

        // Publishes to ROS
        sensor_msgs::PointCloud2::Ptr output (new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*temp_cluster, *output);
        output->header.frame_id = input->header.frame_id;
        pub.publish(output);



    }
      
      
  

}


int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "fusion_size_compare");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/fusion_person_output", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("fusion_correct_person_output", 1);
  pub2 = nh.advertise<sensor_msgs::PointCloud2> ("test_output", 1);

  // Spin
  ros::spin ();
}