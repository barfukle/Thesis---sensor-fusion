#include <ros/ros.h>
#include <ros/console.h>
#include <pcl_ros/publisher.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/radius_outlier_removal.h>

ros::Publisher pub;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // gets kinect cloud
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg(*input, cloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>(cloud));
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_final(new pcl::PointCloud<pcl::PointXYZRGB>);

    /*
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*temp_cloud,*temp_cloud, indices);
    */
    // Color Filtering
    int rMax = 255; // values with icp real time: 255, 100, 70, 0, 100, 0
    int rMin = 100;
    int gMax = 50;
    int gMin = 0;
    int bMax = 50;
    int bMin = 0;
    
    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond (new pcl::ConditionAnd<pcl::PointXYZRGB>());
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::LT, rMax)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::GT, rMin))); 
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::LT, gMax))); 
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::GT, gMin)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::LT, bMax)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::GT, bMin))); 

    pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;
    condrem.setCondition (color_cond);
    condrem.setInputCloud (temp_cloud);
    //condrem.setKeepOrganized(true);

    condrem.filter(*cloud_filtered);
    
    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
    outrem.setInputCloud(cloud_filtered);
    outrem.setRadiusSearch(0.5);
    outrem.setMinNeighborsInRadius(4);
    outrem.filter(*cloud_final);
    



  // Publishes the data
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_final, output);
    pub.publish (output);

}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "kinect_color_filter");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("kinect_voxel_output", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("kinect_cf", 1);

  // Spin
  ros::spin ();
}