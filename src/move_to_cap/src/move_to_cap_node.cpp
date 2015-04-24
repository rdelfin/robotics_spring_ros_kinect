#include <ros/ros.h>
#include <ros/publisher.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include <cmath>

ros::Publisher move_pub;
ros::Time lastCloud;

void goToCentroid(const geometry_msgs::Vector3::ConstPtr&);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_to_cap_node");
    
    ros::NodeHandle node;
    
    ros::Subscriber centroid_sub = node.subscribe("detect_cap/centroid", 100, goToCentroid);
    move_pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 100);
    
    lastCloud = ros::Time::now();
    
    ros::Rate rate(12.0);
    
    while(ros::ok()) {
	
	if((ros::Time::now() - lastCloud).toSec() > 0.5) {
	    geometry_msgs::Twist t;
	    move_pub.publish(t);
	}
	
	ros::spinOnce();
	rate.sleep();
    }
    ros::spin();
}

void goToCentroid(const geometry_msgs::Vector3::ConstPtr& centroid) {
    lastCloud = ros::Time::now();
    
    //Try to obtain
    tf::TransformListener listener;
    tf::StampedTransform transform;
    try{
	ros::Time now = ros::Time(0);
	listener.waitForTransform("base_footprint", "nav_kinect_depth_optical_frame", now, ros::Duration(3.0));
	listener.lookupTransform("base_footprint", "nav_kinect_depth_optical_frame", now, transform);
    } catch (tf::TransformException ex){
	ROS_ERROR("%s",ex.what());
	ros::Duration(1.0).sleep();
	return;
    }
    
    tf::Vector3 centroid_vec(centroid->x, centroid->y, centroid->z);
    tf::Vector3 centroid_transformed = transform(centroid_vec);
    
    geometry_msgs::Twist movement;
    movement.linear.x = 0;
    movement.linear.y = 0;
    movement.linear.z = 0;
    movement.angular.x = 0;
    movement.angular.y = 0;
    movement.angular.z = 0;
    
    if(centroid_transformed.length() > 0.75) {
	centroid_transformed = centroid_transformed.normalized();
	movement.linear.x = centroid_transformed.x()*0.5;
	movement.angular.x = atan2(centroid_transformed.y(), centroid_transformed.x());
	move_pub.publish(movement);
    }
    
    move_pub.publish(movement);
}
