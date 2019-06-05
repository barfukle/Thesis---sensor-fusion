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
ros::Publisher test;

void segment(const sensor_msgs::PointCloud2ConstPtr& input)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*input, cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>(cloud));

    pcl::PassThrough<pcl::PointXYZ> passthru;
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground (new pcl::PointCloud<pcl::PointXYZ>);  
    passthru.setInputCloud(temp_cloud);
    passthru.setFilterFieldName("z");
    passthru.setFilterLimits(-5, 0);
    passthru.filter(*ground);

    sensor_msgs::PointCloud2::Ptr output_test (new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*ground, *output_test);
    output_test->header.frame_id = input->header.frame_id;
    test.publish (output_test);

    pcl::PointIndices::Ptr indices_internal (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(30);
    seg.setDistanceThreshold(0.01); 
    pcl::ModelCoefficients::Ptr coeff (new pcl::ModelCoefficients);
    Eigen::Vector3f axis;
    axis << 0, 0, 1;
    seg.setAxis(axis);
    seg.setInputCloud(ground);
    seg.segment(*indices_internal, *coeff);

    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr it_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr subset_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    
    
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(temp_cloud);
    extract.setIndices(indices_internal);
    extract.setNegative (false);
    extract.filter(*it_cloud);
    *plane_cloud = *it_cloud;
    extract.setNegative (true);
    extract.filter(*subset_cloud);
    temp_cloud.swap(subset_cloud);

    /*
    int i = 0, nr_points = (int) temp_cloud->points.size(); 
    while (temp_cloud->points.size() > 0.4 * nr_points){
        seg.setInputCloud(ground);
        seg.segment(*indices_internal, *coeff);

        pcl::ExtractIndices<pcl::PointXYZ> extract;
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
    */
    
    
    
    sensor_msgs::PointCloud2::Ptr output (new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*plane_cloud, *output);
    output->header.frame_id = input->header.frame_id;
    pub.publish (output);

    sensor_msgs::PointCloud2 output2;
    pcl::toROSMsg(*subset_cloud, output2);
    pub2.publish (output2);

      
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "lidar_segmentation");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/solo_voxel_lidar", 1, segment);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("lidar_plane_output", 1);
  pub2 = nh.advertise<sensor_msgs::PointCloud2> ("lidar_seg_output", 1);
  test = nh.advertise<sensor_msgs::PointCloud2> ("test_seg_output", 1);

  // Spin
  ros::spin ();
}








