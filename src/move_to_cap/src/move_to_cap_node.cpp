#include "ros/ros.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"

//ros::Publisher move_pub;

void goToCentroid(const geometry_msgs::Vector3::ConstPtr&);

int main(int argc, char** argv)
{
    // /cmd_vel_mux/input/teleop
    ros::init(argc, argv, "move_to_cap_node");
    
    ros::NodeHandle nh;
    
    ros::Subscriber centroid_sub = nh.subscribe("detect_cap/centroid", 100, goToCentroid);
    //move_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 100);
    
    ros::Rate rate(10.0);
    while (nh.ok()){
	ros::spinOnce();
	rate.sleep();
   }
}

void goToCentroid(const geometry_msgs::Vector3::ConstPtr& centroid) {
    //Try to obtain
    tf::TransformListener listener;
    tf::StampedTransform transform;
    try{
	ros::Time now = ros::Time::now();
	listener.waitForTransform("base_footprint", "nav_kinect_depth_optical_frame", now, ros::Duration(3.0));
	listener.lookupTransform("base_footprint", "nav_kinect_depth_optical_frame", now, transform);
    } catch (tf::TransformException ex){
	ROS_ERROR("%s",ex.what());
	ros::Duration(1.0).sleep();
	return;
    }
    
    tf::Vector3 centroid_vec(centroid->x, centroid->y, centroid->z);
    tf::Vector3 centroid_transformed = transform(centroid_vec);
    
    ROS_INFO("ORIGINAL CENTROID:    (%.5f, %.5f, %.5f)", centroid->x, centroid->y, centroid->z);
    ROS_INFO("TRANSFORMED CENTROID: (%.5f, %.5f, %.5f)", centroid_transformed.getX(), centroid_transformed.getY(), centroid_transformed.getZ());
}
