#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include "std_msgs/Bool.h"
#include <iostream>
#include <ctime>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <pcl/common/centroid.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/search/pcl_search.h>
#include <pcl/common/transforms.h>

#include <boost/format.hpp>

#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>
#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>

ros::Publisher pub;
ros::Publisher pub2;

using namespace pcl::tracking;

typedef pcl::PointXYZRGBA RefPointType;
typedef ParticleXYZRPY ParticleT;
typedef pcl::PointCloud<pcl::PointXYZRGBA> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;
typedef ParticleFilterTracker<RefPointType, ParticleT> ParticleFilter;

boost::mutex mtx_;
boost::shared_ptr<ParticleFilter> tracker_;
bool model_ready = false;


void track_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    
    if (model_ready == true) {
        // Gets Input Cloud
        pcl::PCLPointCloud2 input_cloud;
	    CloudPtr cloud(new Cloud);

        pcl_conversions::toPCL(*input, input_cloud);
	    pcl::fromPCLPointCloud2(input_cloud, *cloud);
	    boost::mutex::scoped_lock lock (mtx_);
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud,*cloud, indices);

  	    //Tracking
	    tracker_->setInputCloud (cloud);
	    tracker_->compute ();
	    ParticleT positionstate = tracker_->getResult();
	    Eigen::Affine3f movement = tracker_->toEigenMatrix(positionstate);

        //std::cout << movement.matrix() << std::endl;

	    geometry_msgs::Vector3 movement_message;
	    movement_message.x = movement.translation()[0];
	    movement_message.y = movement.translation()[1];
	    movement_message.z = movement.translation()[2];
	    pub.publish(movement_message);

        // Gives XYZ coordinates of person
        std::cout << movement_message << std::endl;
        
        // Visualization of the Point Cloud
        ParticleFilter::PointCloudStatePtr particles = tracker_->getParticles();

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr particle_cloud (new pcl::PointCloud<pcl::PointXYZRGBA> ());
        for (size_t i = 0; i < particles->points.size(); i++)
        {
            pcl::PointXYZRGBA point;
            point.x = particles->points[i].x;
	        point.y = particles->points[i].y;
	        point.z = particles->points[i].z;
	        particle_cloud->points.push_back (point);
        }


        for (int j = 0; j < particle_cloud->points.size(); j++)
        {
            particle_cloud->points[j].r = 255;
            particle_cloud->points[j].g = 0;
            particle_cloud->points[j].b = 0;
        }

        ParticleXYZRPY result = tracker_->getResult ();
        Eigen::Affine3f transformation = tracker_->toEigenMatrix (result);
        transformation.translation () += Eigen::Vector3f (0.0f, 0.0f, 0.0f);
        CloudPtr result_cloud (new Cloud ());
        pcl::transformPointCloud<RefPointType> (*(tracker_->getReferenceCloud ()), *result_cloud, transformation);

        for (int k = 0; k < result_cloud->points.size(); k++)
        {
            result_cloud->points[k].r = 0;
            result_cloud->points[k].g = 0;
            result_cloud->points[k].b = 255;
            result_cloud->points[k].a = 255;
        }
        
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input_conv (new pcl::PointCloud<pcl::PointXYZRGBA> ());
        pcl::fromROSMsg(*input, *input_conv);

        for (int k = 0; k < input_conv->points.size(); k++)
        {
            input_conv->points[k].r = 255;
            input_conv->points[k].g = 255;
            input_conv->points[k].b = 255;
            input_conv->points[k].a = 255;
        }
        
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr image_cloud (new pcl::PointCloud<pcl::PointXYZRGBA> ());
        *image_cloud = *particle_cloud + *input_conv;

        sensor_msgs::PointCloud2::Ptr output (new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*image_cloud, *output);
        output->header.frame_id = input->header.frame_id;
        pub2.publish(output);
        
    }
    
    


}

void model_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    // Gets model for object tracking
    pcl::PCLPointCloud2 input_cloud;
	CloudPtr cloud(new Cloud);
    pcl_conversions::toPCL(*input, input_cloud);
	pcl::fromPCLPointCloud2(input_cloud, *cloud);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud,*cloud, indices);

    Eigen::Vector4f c;
    Eigen::Affine3f trans = Eigen::Affine3f::Identity ();
    CloudPtr transed_ref (new Cloud);

    // Centroid
    pcl::compute3DCentroid<RefPointType> (*cloud, c);
    
    trans.translation ().matrix () = Eigen::Vector3f (c[0], c[1], c[2]);
    pcl::transformPointCloud<RefPointType> (*cloud, *transed_ref, trans.inverse());
    tracker_->setReferenceCloud (transed_ref);
    tracker_->setTrans (trans);
  
    model_ready = true;
    
}


int main (int argc, char** argv)
{
    
    
    std::vector<double> default_step_covariance = std::vector<double> (6, 0.015 * 0.015);
    default_step_covariance[3] *= 40.0;
	default_step_covariance[4] *= 40.0;
	default_step_covariance[5] *= 40.0;

    std::vector<double> initial_noise_covariance = std::vector<double> (6, 0.00001);
	std::vector<double> default_initial_mean = std::vector<double> (6, 0.0);

    boost::shared_ptr<KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> > tracker
	    (new KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> (8));
    
    // Particle Parameters
    ParticleT bin_size;
	bin_size.x = 0.1f;
	bin_size.y = 0.1f;
	bin_size.z = 0.1f;
	bin_size.roll = 0.1f;
	bin_size.pitch = 0.1f;
	bin_size.yaw = 0.1f;
	tracker->setMaximumParticleNum (1000);
	tracker->setDelta (0.99);
	tracker->setEpsilon (0.2);
	tracker->setBinSize (bin_size);
    tracker_ = tracker;
	tracker_->setTrans (Eigen::Affine3f::Identity ());
	tracker_->setStepNoiseCovariance (default_step_covariance);
	tracker_->setInitialNoiseCovariance (initial_noise_covariance);
	tracker_->setInitialNoiseMean (default_initial_mean);
	tracker_->setIterationNum (1);
	tracker_->setParticleNum (600);
	tracker_->setResampleLikelihoodThr(0.00);
	tracker_->setUseNormal (false);
    
    // Creates Coherence

	//ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr coherence = ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr
	    //(new ApproxNearestPairPointCloudCoherence<RefPointType> ());
    ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr coherence (new ApproxNearestPairPointCloudCoherence<RefPointType> ());

	//boost::shared_ptr<DistanceCoherence<RefPointType> > distance_coherence
	    //= boost::shared_ptr<DistanceCoherence<RefPointType> > (new DistanceCoherence<RefPointType> ());
	DistanceCoherence<RefPointType>::Ptr distance_coherence (new DistanceCoherence<RefPointType> ());
    
    HSVColorCoherence<RefPointType>::Ptr color_coherence (new HSVColorCoherence<RefPointType> ());
 
	coherence->addPointCoherence (distance_coherence);
    coherence->addPointCoherence (color_coherence);

	pcl::search::Octree<RefPointType>::Ptr search (new pcl::search::Octree<RefPointType> (0.01));
	coherence->setSearchMethod (search);
	coherence->setMaximumDistance (0.06); 

	tracker_->setCloudCoherence (coherence);
    
  
  
  
  
    // Initialize ROS
    ros::init (argc, argv, "kinect_tracking");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/kinect_voxel_output", 1, track_cb);
    ros::Subscriber sub_model = nh.subscribe ("/kinect_person_output", 1, model_cb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<geometry_msgs::Vector3> ("/movement", 1);
    pub2 = nh.advertise<sensor_msgs::PointCloud2> ("particle_output", 1);

    // Spin
    ros::spin ();
}