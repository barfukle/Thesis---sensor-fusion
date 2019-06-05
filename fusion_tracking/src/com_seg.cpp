#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/angles.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/passthrough.h>

ros::Publisher pub;
ros::Publisher pub2;
ros::Publisher test1;

void segment(const sensor_msgs::PointCloud2ConstPtr& input)
{
    // Gets ICP Cloud
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg(*input, cloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>(cloud));

    /*
    // Pass Through Filter
    pcl::PassThrough<pcl::PointXYZRGB> passthru;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tester(new pcl::PointCloud<pcl::PointXYZRGB>());
    passthru.setInputCloud(temp_cloud);
    passthru.setFilterFieldName("y");
    passthru.setFilterLimits(0.75, 2); //0.75,2
    passthru.setNegative(true);
    passthru.filter(*tester);

    sensor_msgs::PointCloud2 output3;
    pcl::toROSMsg(*tester, output3);
    output3.header.frame_id = input->header.frame_id;
    test1.publish (output3);
    */

    // Segmentation by RANSAC
    pcl::PointIndices::Ptr indices_internal (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.01);
    pcl::ModelCoefficients::Ptr coeff (new pcl::ModelCoefficients);
    seg.setAxis(Eigen::Vector3f(0,1,0));
    seg.setEpsAngle(0.2);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr it_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr subset_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    
    int i = 0, nr_points = (int) temp_cloud->points.size(); 
    while (temp_cloud->points.size() > 0.8 * nr_points){
        seg.setInputCloud(temp_cloud);
        seg.segment(*indices_internal, *coeff);

        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(temp_cloud);
        extract.setIndices(indices_internal);
        extract.setNegative (false);
        extract.filter(*it_cloud);
        *plane_cloud = *plane_cloud + *it_cloud;

        extract.setNegative (true);
        extract.filter(*subset_cloud);
        temp_cloud.swap(subset_cloud);
        i++;
    }
    
    
    // Publishes to ROS, both segmented cloud and indices
    /*
    sensor_msgs::PointCloud2::Ptr output (new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*plane_cloud, *output);
    output->header.frame_id = input->header.frame_id;
    pub.publish (output);
    */
    sensor_msgs::PointCloud2 output2;
    pcl::toROSMsg(*temp_cloud, output2);
    output2.header.frame_id = input->header.frame_id;
    pub2.publish (output2);
    
      
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "fusion_seg");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/icp_output", 1, segment);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("plane_output", 1);
  pub2 = nh.advertise<sensor_msgs::PointCloud2> ("seg_output", 1);
  test1 = nh.advertise<sensor_msgs::PointCloud2> ("seg_test_output", 1);

  // Spin
  ros::spin ();
}








