#include "ros/ros.h"
#include <tf/tf.h>
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"

ros::Publisher move_pub;

void goToCentroid(const geometry_msgs::Vector3::ConstPtr&);

int main(int argc, char** argv)
{
    // /cmd_vel_mux/input/teleop
    ros::init(argc, argv, "move_to_cap_node");
    
    ros::NodeHandle nh;
    
    ros::Subscriber centroid_sub = nh.subscribe("detect_cap/centroid", 100, goToCentroid);
    tf::TransformListener listener;
    move_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 100);
    
    ros::Rate rate(10.0);
    while (nh.ok()){
	ros::spinOnce();
	rate.sleep();
   }
}

void goToCentroid(const geometry_msgs::Vector3::ConstPtr& centroid) {
    //Try to obtain
    tf::StampedTransform transform;
    try{
	listener.lookupTransform("/base_link", "/camera_depth_optical_frame", ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
	ROS_ERROR("%s",ex.what());
	ros::Duration(1.0).sleep();
	return;
    }
    
    tf::Vector3 centroid_vec(centroid->x, centroid->y, centroid->z);
    tf::Vector3 centroid_transformed = vectorTransform(centroid_vec);
    
    ROS_INFO("TRANSFORMED CENTROID: " + centroid_transformed.x, centroid_transformed.y, centroid_transformed.z);
}
