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


class pos_seg
{

private:

ros::Publisher pub;
ros::Publisher pub2;
ros::Subscriber sub1;
ros::Subscriber sub2;
ros::NodeHandle nh;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr kinect_color;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr icp_cloud;
float min_x;
float max_x;
float min_y;
float max_y;
float min_z;
float max_z;

public:

  pos_seg() :
  kinect_color(new pcl::PointCloud<pcl::PointXYZRGB>),
  icp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>)

  {

    pub = nh.advertise<sensor_msgs::PointCloud2> ("segment_icp", 1);
    pub2 = nh.advertise<sensor_msgs::PointCloud2> ("test_output", 1);

    sub1 = nh.subscribe("/color_filter_output", 1, &pos_seg::findColorObject, this);
    sub2 = nh.subscribe ("/icp_output", 1, &pos_seg::takeICPSegment, this);

  }
  
  void findColorObject(const sensor_msgs::PointCloud2ConstPtr& color_input)
  {
    
    pcl::fromROSMsg(*color_input, *kinect_color);
    
    /*
    float min_x = kinect_color->points[0].x;
    float max_x = kinect_color->points[0].x;
    float min_y = kinect_color->points[0].y;
    float max_y = kinect_color->points[0].y;
    float min_z = kinect_color->points[0].z;
    float max_z = kinect_color->points[0].z;
    */

    // Creates Box Ranges
    min_x = kinect_color->points[0].x;
    max_x = kinect_color->points[0].x;
    min_y = kinect_color->points[0].y;
    max_y = kinect_color->points[0].y;
    min_z = kinect_color->points[0].z;
    max_z = kinect_color->points[0].z;

    for (size_t i = 0; i < kinect_color->points.size(); i++)
    {
        if (kinect_color->points[i].x <= min_x)
        {
            min_x = kinect_color->points[i].x;
        }
        else if (kinect_color->points[i].y <= min_y)
        {
            min_y = kinect_color->points[i].y;
        }
        else if (kinect_color->points[i].z <= min_z)
        {
            min_z = kinect_color->points[i].z;
        }
        else if (kinect_color->points[i].x >= max_x)
        {
            max_x = kinect_color->points[i].x;
        }
        else if (kinect_color->points[i].y >= max_y)
        {
            max_y = kinect_color->points[i].y;
        }
        else if (kinect_color->points[i].z >= max_z)
        {
            max_z = kinect_color->points[i].z;
            
        }
        
    }
    /*
    std::cout << "Min x: " << min_x << std::endl;
    std::cout << "Min y: " << min_y << std::endl;
    std::cout << "Min z: " << min_z << std::endl;
    std::cout << "Max x: " << max_x << std::endl;
    std::cout << "Max y: " << max_y << std::endl;
    std::cout << "Max z: " << max_z << std::endl;
    */
    
    sensor_msgs::PointCloud2::Ptr output (new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*kinect_color, *output);
    output->header.frame_id = color_input->header.frame_id;
    pub2.publish (output);
    
    
  }

  

  void takeICPSegment(const sensor_msgs::PointCloud2ConstPtr& icp_input)
  {

    pcl::fromROSMsg(*icp_input, *icp_cloud);
    
    /*
    std::cout << "Min x: " << min_x << std::endl;
    std::cout << "Min y: " << min_y << std::endl;
    std::cout << "Min z: " << min_z << std::endl;
    std::cout << "Max x: " << max_x << std::endl;
    std::cout << "Max y: " << max_y << std::endl;
    std::cout << "Max z: " << max_z << std::endl;
    */



    // Gets all Points within the Box
    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::GT, min_z)));
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::LT, max_z)));
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("y", pcl::ComparisonOps::GT, min_y)));
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("y", pcl::ComparisonOps::LT, max_y)));
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("x", pcl::ComparisonOps::GT, min_x)));
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("x", pcl::ComparisonOps::LT, max_x)));

    pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;
    condrem.setCondition(range_cond);
    condrem.setInputCloud(icp_cloud);
    //condrem.setKeepOrganized(true);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    condrem.filter(*filtered_cloud);
    
    //Publishes Output
    sensor_msgs::PointCloud2::Ptr output (new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*filtered_cloud, *output);
    output->header.frame_id = icp_input->header.frame_id;
    pub.publish (output);
    

  }


};



int
main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "fusion_posseg");

    pos_seg first;
    
  
    ros::spin ();
    return 0;
}