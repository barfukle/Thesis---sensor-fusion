#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>    
#include <pcl/io/openni_grabber.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/people/ground_based_people_detection_app.h>
#include <pcl/common/time.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>
#include <visualization_msgs/Marker.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/people/person_cluster.h>
#include <pcl/PointIndices.h>

//using namespace std::chrono_literals;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

boost::mutex cloud_mutex;
ros::Publisher pub;
ros::Publisher test1;
ros::Publisher test2;
ros::Publisher marker_pub;

Eigen::VectorXf ground_coeffs;
pcl::people::GroundBasedPeopleDetectionApp<PointT> people_detector;
pcl::people::PersonClassifier<pcl::RGB> person_classifier;
std::string svm_filename = "../Downloads/trainedLinearSVMForPeopleDetectionWithHOG.yaml";
/*
visualization_msgs::Marker mark_cluster(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster, Eigen::Vector3f centroid, Eigen::Vector3f min, Eigen::Vector3f max,int id, float r, float g, float b)
{
    
    

    uint32_t shape = visualization_msgs::Marker::CUBE;
    visualization_msgs::Marker marker;
    marker.header.frame_id = cloud_cluster->header.frame_id;
    marker.header.stamp = ros::Time::now();

    marker.ns = "person";
    marker.id = id;
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = centroid[0];
    marker.pose.position.y = centroid[1];
    marker.pose.position.z = centroid[2];
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = (max[0]-min[0]);
    marker.scale.y = (max[1]-min[1]);
    marker.scale.z = (max[2]-min[2]);

    if (marker.scale.x ==0)
        marker.scale.x=0.1;

    if (marker.scale.y ==0)
        marker.scale.y=0.1;

    if (marker.scale.z ==0)
        marker.scale.z=0.1;

    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 0.5;

    marker.lifetime = ros::Duration();
    //   marker.lifetime = ros::Duration(0.5);
    return marker;
    //marker_pub.publish(marker);
}
*/
void GroundFinder (const sensor_msgs::PointCloud2ConstPtr& input)
{
    pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ground (new pcl::PointCloud<pcl::PointXYZRGBA>);  
    pcl::fromROSMsg (*input, cloud);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>(cloud));

    std::vector<int> nan_indices;
    pcl::removeNaNFromPointCloud(*input_cloud,*input_cloud, nan_indices);

    cloud_mutex.lock();
    /*
    pcl::PassThrough<pcl::PointXYZRGBA> passthru;

    passthru.setInputCloud(input_cloud);
    passthru.setFilterFieldName("y");
    passthru.setFilterLimits(0, 2);
    passthru.filter(*ground);

    sensor_msgs::PointCloud2::Ptr output (new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*ground, *output);
    output->header.frame_id = input->header.frame_id;
    test1.publish(output);
    
    sensor_msgs::PointCloud2::Ptr output2 (new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*input_cloud, *output2);
    output2->header.frame_id = input->header.frame_id;
    test2.publish(output2);
    */

    sensor_msgs::PointCloud2::Ptr output (new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*input_cloud, *output);
    output->header.frame_id = input->header.frame_id;
    test1.publish(output);
    

    float min_y = input_cloud->points[0].y;

    for (size_t i = 0; i < input_cloud->points.size(); i++)
    {
        
        if (input_cloud->points[i].y >= min_y)
        {
            min_y = input_cloud->points[i].y;
        }        
        
    }

    //std::cout << "min y: " << min_y << std::endl;

    float decision = min_y - 0.02;
    bool points_full = false;
    PointCloudT::Ptr chosen_points (new PointCloudT);
    
    
    
    //std::cout << "decision: " << decision << std::endl;
    
    while (!points_full)
    {
        
        
        while (chosen_points->size() <= 2)
        {

            

            for (size_t j = 0; j < input_cloud->points.size(); j++)
            {
            
                
                if (input_cloud->points[j].y > decision)
                {
                    
                    chosen_points->points.push_back(input_cloud->points[j]);
                    
                
                }        
                
                
                
            }

            
            

        }
        
        if (chosen_points->size() > 2) 
        {
            points_full = true;
        }
        
    }

    
    //std::cout << "first size: " << chosen_points->size() << std::endl;

    PointCloudT::Ptr three_points (new PointCloudT);

    if (chosen_points->size() >= 4)
    {
        for (size_t k = 0; k < 3; k++)
        {
                        
            three_points->points.push_back(chosen_points->points[k]); 
                               
        }
    }
    else if (chosen_points->size() == 3)
    {

        three_points = chosen_points;

    }

    //std::cout << "second size: " << three_points->size() << std::endl;

    PointCloudT::Ptr test (new PointCloudT);

    ground_coeffs.resize(4);
    std::vector<int> three_points_indices;
    for (unsigned int i = 0; i < three_points->points.size(); i++)
        three_points_indices.push_back(i);
    pcl::SampleConsensusModelPlane<PointT> model_plane(three_points);
    model_plane.computeModelCoefficients(three_points_indices,ground_coeffs);
    model_plane.projectPoints(three_points_indices, ground_coeffs, *test, true);

    //std::cout << "Ground plane: " << ground_coeffs(0) << " " << ground_coeffs(1) << " " << ground_coeffs(2) << " " << ground_coeffs(3) << " " << std::endl;
    
    
    sensor_msgs::PointCloud2::Ptr output2 (new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*test, *output2);
    output2->header.frame_id = input->header.frame_id;
    test2.publish(output2);
    
    
    person_classifier.loadSVMFromFile(svm_filename);  
    people_detector.setVoxelSize(0.06);                                                           
    Eigen::Matrix3f rgb_intrinsics_matrix;
    rgb_intrinsics_matrix << 525, 0.0, 319.5, 0.0, 525, 239.5, 0.0, 0.0, 1.0; 
    people_detector.setIntrinsics(rgb_intrinsics_matrix);                                          
    people_detector.setClassifier(person_classifier);                                                   
    people_detector.setPersonClusterLimits(1.0, 2.3, 0.1, 2.0);  
    
    
    //people detection      
    std::vector<pcl::people::PersonCluster<PointT> > clusters;   
    people_detector.setInputCloud(input_cloud);
    people_detector.setGround(ground_coeffs);                                           
    people_detector.compute(clusters);      
    //update ground
    ground_coeffs = people_detector.getGround();

    

    std::cout << "clusters: " << clusters.size() << std::endl; 
    
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr combined_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr subset_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());

    int i = 0, j = 0;
    for(std::vector<pcl::people::PersonCluster<pcl::PointXYZRGBA> >::iterator it = clusters.begin(); it != clusters.end(); ++it)
    {
        if(it->getPersonConfidence() > -3.5)             
        {
            //Centroid, min and max points
            Eigen::Vector3f centroid = clusters[j].getCenter();
            Eigen::Vector3f minPoints = clusters[j].getMin();
            Eigen::Vector3f maxPoints = clusters[j].getMax();
            //draw a marker
            //mark_cluster(input_cloud, centroid, minPoints, maxPoints,i, 255, 0, 0);
            i++;

            pcl::PointIndices::Ptr indices_internal (new pcl::PointIndices);
            *indices_internal = clusters[j].getIndices();
            pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
            extract.setInputCloud(input_cloud);
            extract.setIndices(indices_internal);
            extract.setNegative (false);
            extract.filter(*subset_cloud);
            *combined_cloud = *combined_cloud + *subset_cloud;
        
            sensor_msgs::PointCloud2::Ptr output3 (new sensor_msgs::PointCloud2);
            pcl::toROSMsg(*combined_cloud, *output3);
            output3->header.frame_id = input->header.frame_id;
            pub.publish(output3);




        }
        
        
        



        j++;
    }
    std::cout << " " << i << " people found" << " " << std::endl;


    cloud_mutex.unlock();
}





int main (int argc, char** argv)
{
    


    // Initialize ROS
    ros::init (argc, argv, "lidar_hog");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/velodyne_points", 1, GroundFinder);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2> ("/lidar_hog_output", 1);
    test1 = nh.advertise<sensor_msgs::PointCloud2> ("/test1_output", 1);
    test2 = nh.advertise<sensor_msgs::PointCloud2> ("/test2_output", 1);
    //marker_pub = nh.advertise<visualization_msgs::Marker> ("/marker_output", 1);

    // Spin
    ros::spin ();
}