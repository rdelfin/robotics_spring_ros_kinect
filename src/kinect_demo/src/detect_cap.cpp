#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Vector3.h"
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <cstdio>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <cmath>

// camera_link for fixed frame in global option in rviz

/* define what kind of point clouds we're using */
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

bool new_cloud_available_flag = false;

// General point cloud to store the whole image
PointCloudT::Ptr cloud (new PointCloudT);

//Point Cloud to store out neon cap
PointCloudT::Ptr neon_cloud (new PointCloudT);

// Message required to publish the cloud - Convert from pcl to msg
sensor_msgs::PointCloud2 cloud_ros;


void cloud_sub(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    //convert the msg to PCL format
    pcl::fromROSMsg (*msg, *cloud);

    //state that a new cloud is available
    new_cloud_available_flag = true;

    /**PointCloudT::iterator myIterator;
    for(myIterator = cloud->begin();
	    myIterator != cloud->end();
	    myIterator++)
    {
	    std::cout<<*myIterator<<" ";
    }**/
}

PointCloudT::Ptr computeNeonVoxels(PointCloudT::Ptr in) {
    int total_neon = 0;

    //Point Cloud to store out neon cap
	PointCloudT::Ptr temp_neon_cloud (new PointCloudT);

    for (int i = 0; i < in->points.size(); i++) {
	unsigned int r, g, b;
	r = in->points[i].r;
	g = in->points[i].g;
	b = in->points[i].b;
	// Look for mostly neon value points
	//146,81,20
	if (g > 100 && r < 100 && b < 70) {
	    temp_neon_cloud->push_back(in->points[i]);
	}
    }

    return temp_neon_cloud;
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "kinect_fun");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/nav_kinect/depth_registered/points", 1000, cloud_sub);

    //debugging publisher --> can create your own topic and then subscribe to it through rviz
    ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("detect_cap/cloud", 100);
    ros::Publisher centroid_pub = nh.advertise<geometry_msgs::Vector3>("detect_cap/centroid", 100);

    //refresh rate
    double ros_rate = 3.0;
    ros::Rate r(ros_rate);

    while (ros::ok())
    {
	ros::spinOnce();
	r.sleep();

	if (new_cloud_available_flag){
	    new_cloud_available_flag = false;

	    // Voxel Grid reduces the computation time. Its a good idea to do it if you will be doing
	    //sequential processing or frame-by-frame
	    // Create the filtering object: downsample the dataset using a leaf size of 1cm
	    pcl::VoxelGrid<PointT> vg;
	    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
	    vg.setInputCloud (cloud);
	    vg.setLeafSize (0.005f, 0.005f, 0.005f);
	    vg.filter (*cloud_filtered);

	    int max_num_neon = 0;

	    //Send the filtered point cloud to be processed in order to get the neon blob
	    neon_cloud = computeNeonVoxels(cloud_filtered);

	    //Publish the cloud with the neon cap
	    pcl::toROSMsg(*neon_cloud,cloud_ros);

	    //Set the frame ID to the first cloud we took in coz we want to replace that one
	    cloud_ros.header.frame_id = cloud->header.frame_id;
	    cloud_pub.publish(cloud_ros);

	    // Find the centroid of the neon cap
	    Eigen::Vector4f centroid;
	    pcl::compute3DCentroid(*neon_cloud, centroid);
	    geometry_msgs::Vector3 centroid_msg;
	    
	    ROS_INFO("Neon cloud size: %ld", neon_cloud->points.size());
	    
	    //Same as checking if none are not a number
	    if(!isnan(centroid(0)) && !isnan(centroid(1)) && !isnan(centroid(2))) {
		centroid_msg.x = centroid(0);
		centroid_msg.y = centroid(1);
		centroid_msg.z = centroid(2);
		centroid_pub.publish(centroid_msg);
	    }
	    
	    ROS_INFO("The centroid of the neon cap is: (%f, %f, %f)", centroid(0), centroid(1), centroid(2));

	}
    }

    return 0;
}
