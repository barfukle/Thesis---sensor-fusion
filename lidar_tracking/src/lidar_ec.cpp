#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
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
#include <pcl_ros/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <boost/lexical_cast.hpp>
#include <pcl/filters/conditional_removal.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

ros::Publisher pub;
ros::Publisher pub2;
//std::vector<ros::Publisher> pub_vec;


void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Input Cloud
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*input, cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>(cloud));

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (temp_cloud);
  // Euclidean Clustering
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.1);   //real time = 0.1
  ec.setMinClusterSize (150);     //real time = 150
  //ec.setMaxClusterSize (1500);
  ec.setSearchMethod (tree);
  ec.setInputCloud (temp_cloud);
  ec.extract (cluster_indices);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud (new pcl::PointCloud<pcl::PointXYZ>());
  // Cluster Identification
  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        cloud_cluster->points.push_back (temp_cloud->points[*pit]); 
      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;
      /*
      sensor_msgs::PointCloud2::Ptr output (new sensor_msgs::PointCloud2);
      pcl::toROSMsg(*cloud_cluster, *output);
      output->header.frame_id = input->header.frame_id;
      pub.publish(output);
      */

      *combined_cloud = *cloud_cluster + *combined_cloud;
      sensor_msgs::PointCloud2::Ptr output2 (new sensor_msgs::PointCloud2);
      pcl::toROSMsg(*combined_cloud, *output2);
      output2->header.frame_id = input->header.frame_id;
      pub2.publish(output2);

      float min_x = cloud_cluster->points[0].x;
      float max_x = cloud_cluster->points[0].x;
      float min_y = cloud_cluster->points[0].y;
      float max_y = cloud_cluster->points[0].y;
      float min_z = cloud_cluster->points[0].z;
      float max_z = cloud_cluster->points[0].z;

      for (size_t i = 0; i < cloud_cluster->points.size(); i++)
      {
        if (cloud_cluster->points[i].x <= min_x)
          {
            min_x = cloud_cluster->points[i].x;
          }
        else if (cloud_cluster->points[i].y <= min_y)
          {
            min_y = cloud_cluster->points[i].y;
          }
        else if (cloud_cluster->points[i].z <= min_z)
          {
            min_z = cloud_cluster->points[i].z;
          }
        else if (cloud_cluster->points[i].x >= max_x)
          {
            max_x = cloud_cluster->points[i].x;
          }
        else if (cloud_cluster->points[i].y >= max_y)
          {
            max_y = cloud_cluster->points[i].y;
          }
        else if (cloud_cluster->points[i].z >= max_z)
          {
            max_z = cloud_cluster->points[i].z;
            
          }
        
      }

      if ( max_y - min_y < 1.0 && max_x - min_x < 0.7)
      {
          sensor_msgs::PointCloud2::Ptr output (new sensor_msgs::PointCloud2);
          pcl::toROSMsg(*cloud_cluster, *output);
          output->header.frame_id = input->header.frame_id;
          pub.publish(output);


      }


    }
  
  

}


int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "lidar_ec");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/lidar_sparse_output", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("lidar_person_output", 1);
  pub2 = nh.advertise<sensor_msgs::PointCloud2> ("lidar_all_cluster_output", 1);

  // Spin
  ros::spin ();
}