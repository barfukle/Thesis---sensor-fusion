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
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <boost/lexical_cast.hpp>
#include <pcl/filters/conditional_removal.h>

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
    sensor_msgs::PointCloud2::Ptr output (new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*input_cloud, *output);
    output->header.frame_id = input->header.frame_id;
    test1.publish(output);
    */

    // Finds the Ground Plane Coefficients
    pcl::PassThrough<pcl::PointXYZRGBA> passthru;
    passthru.setInputCloud(input_cloud);
    passthru.setFilterFieldName("y");
    passthru.setFilterLimits(0.5, 2);
    passthru.filter(*ground);
    /*
    sensor_msgs::PointCloud2::Ptr output2 (new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*ground, *output2);
    output2->header.frame_id = input->header.frame_id;
    test2.publish(output2);
    */
    ground_coeffs.resize(4);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    Eigen::Vector3f axis;
    axis << 0, 0, 1;
    seg.setAxis(axis);
    seg.setDistanceThreshold (0.1);
    seg.setInputCloud (ground);
    seg.segment (*inliers, *coefficients);
    ground_coeffs[0]= coefficients->values[0]; ground_coeffs[1]= coefficients->values[1];
    ground_coeffs[2]= coefficients->values[2]; ground_coeffs[3]= coefficients->values[3];

    //std::cout << "Ground plane: " << ground_coeffs(0) << " " << ground_coeffs(1) << " " << ground_coeffs(2) << " " << ground_coeffs(3) << " " << std::endl;
    
    // Shows Ground Plane
    PointCloudT::Ptr test (new PointCloudT);
    pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
    extract.setInputCloud(ground);
    extract.setIndices(inliers);
    extract.setNegative (false);
    extract.filter(*test);
    sensor_msgs::PointCloud2::Ptr output2 (new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*test, *output2);
    output2->header.frame_id = input->header.frame_id;
    test2.publish(output2);
    
    // Sets Parameters for the People Detector
    person_classifier.loadSVMFromFile(svm_filename);  
    people_detector.setVoxelSize(0.06);                                                           
    Eigen::Matrix3f rgb_intrinsics_matrix;
    rgb_intrinsics_matrix << 525, 0.0, 319.5, 0.0, 525, 239.5, 0.0, 0.0, 1.0; 
    people_detector.setIntrinsics(rgb_intrinsics_matrix);                                          
    people_detector.setClassifier(person_classifier);                                                   
    people_detector.setPersonClusterLimits(1.0, 2.3, 0.1, 8);  
    
    
    // People Detection    
    std::vector<pcl::people::PersonCluster<PointT> > clusters;   
    people_detector.setInputCloud(input_cloud);
    people_detector.setGround(ground_coeffs);                                           
    people_detector.compute(clusters);      
    ground_coeffs = people_detector.getGround();
    
    //std::cout << "clusters: " << clusters.size() << std::endl; 
    
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr combined_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr subset_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr visual_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());

    int i = 0, j = 0;
    for(std::vector<pcl::people::PersonCluster<pcl::PointXYZRGBA> >::iterator it = clusters.begin(); it != clusters.end(); ++it)
    {
        // Finds best confidence clusters
        if(it->getPersonConfidence() > -5)             
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
            
            *visual_cloud = *visual_cloud + *subset_cloud;

            sensor_msgs::PointCloud2::Ptr output2 (new sensor_msgs::PointCloud2);
            pcl::toROSMsg(*subset_cloud, *output2);
            output2->header.frame_id = input->header.frame_id;
            test2.publish(output2);
            
            // Color Filtering
            int rMax = 255;
            int rMin = 100;
            int gMax = 50;
            int gMin = 0;
            int bMax = 50;
            int bMin = 0;
    
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr color_comp_cluster (new pcl::PointCloud<pcl::PointXYZRGBA>);
            pcl::ConditionAnd<pcl::PointXYZRGBA>::Ptr color_cond (new pcl::ConditionAnd<pcl::PointXYZRGBA>());
            color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGBA>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGBA> ("r", pcl::ComparisonOps::LT, rMax)));
            color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGBA>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGBA> ("r", pcl::ComparisonOps::GT, rMin))); 
            color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGBA>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGBA> ("g", pcl::ComparisonOps::LT, gMax))); 
            color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGBA>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGBA> ("g", pcl::ComparisonOps::GT, gMin)));
            color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGBA>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGBA> ("b", pcl::ComparisonOps::LT, bMax)));
            color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGBA>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGBA> ("b", pcl::ComparisonOps::GT, bMin))); 

            pcl::ConditionalRemoval<pcl::PointXYZRGBA> condrem;
            condrem.setCondition (color_cond);
            condrem.setInputCloud (subset_cloud);
            //condrem.setKeepOrganized(true);

            condrem.filter(*color_comp_cluster);

            if (color_comp_cluster->size() > 10) {
            /*
            sensor_msgs::PointCloud2::Ptr output (new sensor_msgs::PointCloud2);
            pcl::toROSMsg(*color_comp_cluster, *output);
            output->header.frame_id = input->header.frame_id;
            pub.publish(output);
            */

            *combined_cloud = *subset_cloud;
        

            }


        }
        
        sensor_msgs::PointCloud2::Ptr output3 (new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*combined_cloud, *output3);
        output3->header.frame_id = input->header.frame_id;
        pub.publish(output3);

        



        j++;
    }
    //std::cout << " " << i << " people found" << " " << std::endl;


    cloud_mutex.unlock();
}





int main (int argc, char** argv)
{
    


    // Initialize ROS
    ros::init (argc, argv, "fusion_hog");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/icp_output", 1, GroundFinder);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2> ("/hog_output", 1);
    test1 = nh.advertise<sensor_msgs::PointCloud2> ("/test1_output", 1);
    test2 = nh.advertise<sensor_msgs::PointCloud2> ("/test2_output", 1);
    //marker_pub = nh.advertise<visualization_msgs::Marker> ("/marker_output", 1);

    // Spin
    ros::spin ();
}