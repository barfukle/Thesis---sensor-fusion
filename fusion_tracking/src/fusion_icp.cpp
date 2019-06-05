#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <pcl_ros/publisher.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl_ros/transforms.h>

class icp
{
private:

ros::Publisher test1;
ros::Publisher test2;
ros::Publisher pub;
ros::Subscriber sub1;
ros::Subscriber sub2;
ros::NodeHandle nh;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr kinect_XYZ;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr lidar_XYZ;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr combined_cloud;

public:

  icp() :
  kinect_XYZ(new pcl::PointCloud<pcl::PointXYZRGB>),
  lidar_XYZ(new pcl::PointCloud<pcl::PointXYZRGB>),
  combined_cloud(new pcl::PointCloud<pcl::PointXYZRGB>)

  {

    pub = nh.advertise<sensor_msgs::PointCloud2> ("icp_output", 1);

    sub1 = nh.subscribe("/voxel_output", 1, &icp::kinect, this);
    sub2 = nh.subscribe ("/voxel_lidar_output", 1, &icp::lidar, this);
    

  }
  
  void kinect(const sensor_msgs::PointCloud2ConstPtr& kinect_input)
  {
    // Gets Kinect Cloud
    pcl::fromROSMsg(*kinect_input, *kinect_XYZ);
    
    
  }

  

  void lidar(const sensor_msgs::PointCloud2ConstPtr& lidar_input)
  {
    // Gets Lidar Cloud
    pcl::fromROSMsg(*lidar_input, *lidar_XYZ);
    
    // Applies Manual Transformation Matrix to shift Lidar to Kinect
    float x = -M_PI/2;  
    float y = M_PI/2;   
    float z = M_PI;     

    Eigen::Matrix4f transform_x = Eigen::Matrix4f::Identity();
    transform_x (1,1) = cos (x);
    transform_x (1,2) = -sin (x);
    transform_x (2,1) = sin (x);
    transform_x (2,2) = cos (x);
    
    Eigen::Matrix4f transform_y = Eigen::Matrix4f::Identity();
    transform_y (0,0) = cos (y);
    transform_y (2,0) = -sin (y);
    transform_y (0,2) = sin (y);
    transform_y (2,2) = cos (y);
    
    Eigen::Matrix4f transform_z = Eigen::Matrix4f::Identity();
    transform_z (0,0) = cos (z);
    transform_z (0,1) = -sin (z);
    transform_z (1,0) = sin (z);
    transform_z (1,1) = cos (z);


    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr lidar_shift(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud(*lidar_XYZ, *lidar_shift, transform_x);
    pcl::transformPointCloud(*lidar_shift, *lidar_shift, transform_z);
    pcl::transformPointCloud(*lidar_shift, *lidar_shift, transform_y);
    /*
    *combined_cloud = *kinect_XYZ + *lidar_shift;

    sensor_msgs::PointCloud2::Ptr output3 (new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*combined_cloud, *output3);
    output3->header.frame_id = kinect_XYZ->header.frame_id;
    pub.publish(output3);
    */


    // ICP Alignment
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr kinect_icp(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::IterativeClosestPoint<pcl::PointXYZRGB,pcl::PointXYZRGB> icp;
    icp.setInputSource(kinect_XYZ);
    icp.setInputTarget(lidar_shift);
    icp.setMaximumIterations (30); 
    icp.setMaxCorrespondenceDistance (0.5);
    icp.setTransformationEpsilon(1e-9); 
    icp.setRANSACOutlierRejectionThreshold(0.5);
    //pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(*kinect_icp);
    //std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
    //std::cout << icp.getFinalTransformation() << std::endl;
    Eigen::Matrix4f transformation = icp.getFinalTransformation();

    /*
    Eigen::Matrix4f transform_new;
    transform_new = transformation;
    transform_new (0,3) = 0;
    transform_new (1,3) = 0;
    transform_new (2,3) = 0;
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr kinect_align(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud(*kinect_XYZ, *kinect_align, transform_new);


    *combined_cloud = *kinect_align + *lidar_shift;
    */
    
    // Combines Aligned Kinect Cloud with Lidar
    *combined_cloud = *kinect_icp + *lidar_shift;


    //std::cout << "\n ICP Original Matrix: \n" << transformation << std::endl;
    //std::cout << "\n ICP Altered Matrix: \n" << transform_new << std::endl;
    
    //pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>(*kinect_XYZ));
    
    // Publishes to ROS
    sensor_msgs::PointCloud2::Ptr output3 (new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*combined_cloud, *output3);
    output3->header.frame_id = kinect_XYZ->header.frame_id;
    pub.publish(output3);
    


    /*
    std::cout << "Total Points from Kinect: " << kinect_XYZ->size() << std::endl;
    std::cout << "Total Points from Lidar: " << lidar_XYZ->size() << std::endl;
    std::cout << "Total Points from Icp: " << combined_cloud->size() << std::endl;
    */
  }


};








int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "fusion_icp");
 
  icp first;

  
  ros::spin ();
  return 0;
}