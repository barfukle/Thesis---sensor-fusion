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
  // Gets Segmented Cloud
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  pcl::fromROSMsg(*input, cloud);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>(cloud));

  // Euclidean Clustering
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (temp_cloud);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance (0.175); //0.175
  ec.setMinClusterSize (50);     //50
  //ec.setMaxClusterSize (2000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (temp_cloud);
  ec.extract (cluster_indices);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr combined_cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);

  // Cluster Identification Loop to find Person
  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        cloud_cluster->points.push_back (temp_cloud->points[*pit]); 
      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      *combined_cloud = *cloud_cluster + *combined_cloud;
      sensor_msgs::PointCloud2::Ptr output2 (new sensor_msgs::PointCloud2);
      pcl::toROSMsg(*combined_cloud, *output2);
      output2->header.frame_id = input->header.frame_id;
      pub2.publish(output2);
      
      // Color Filtering
      int rMax = 255;
      int rMin = 100;
      int gMax = 50;
      int gMin = 0;
      int bMax = 50;
      int bMin = 0;
    
      // Color Comparison/Filter
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_comp_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond (new pcl::ConditionAnd<pcl::PointXYZRGB>());
      color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::LT, rMax)));
      color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::GT, rMin))); 
      color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::LT, gMax))); 
      color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::GT, gMin)));
      color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::LT, bMax)));
      color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::GT, bMin))); 

      pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;
      condrem.setCondition (color_cond);
      condrem.setInputCloud (cloud_cluster);
      //condrem.setKeepOrganized(true);

      condrem.filter(*color_comp_cluster);

      if ( color_comp_cluster->size() > 10 )
      {

          sensor_msgs::PointCloud2::Ptr output (new sensor_msgs::PointCloud2);
          pcl::toROSMsg(*cloud_cluster, *output);
          output->header.frame_id = input->header.frame_id;
          pub.publish(output);

      }
      /*
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
      
      if ( max_z - min_z < 1.0 && max_x - min_x < 1.0) //real time = 1.0, 0.7
      {
          
          sensor_msgs::PointCloud2::Ptr output (new sensor_msgs::PointCloud2);
          pcl::toROSMsg(*temp_cluster, *output);
          output->header.frame_id = input->header.frame_id;
          pub.publish(output);



      }
      */
      






    }
  
  

}


int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "fusion_ec");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/seg_output", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("fusion_person_output", 1);
  pub2 = nh.advertise<sensor_msgs::PointCloud2> ("fusion_combined_output", 1);

  // Spin
  ros::spin ();
}