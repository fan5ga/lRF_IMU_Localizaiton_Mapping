#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "sensor_msgs/Imu.h"


/* callback function   
 * when there is a message arrived, the callback function will be used to handle the message
 * in this callback function, the arrived IMU message will be converted to tf message, then, the function will broadcast a tf message
 * the father frame ID is map, the child frame ID is laser
 */

float x, y;

void poseCallback(const geometry_msgs::Twist::ConstPtr& msg){                     
  static tf::TransformBroadcaster br;
  tf::Transform transform;

  y += msg->linear.x * sin(msg->angular.z/6000)/700;
  x += msg->linear.x * cos(msg->angular.z/6000)/700;
  
  transform.setOrigin( tf::Vector3(x, y, 0) );                                            //set the transform from father frame to child frame
  tf::Quaternion q;
  q.setRPY(0, 0, msg->angular.z/6000);				  //change the Euler angle to quaternion
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));    //broadcast the tf message
}

int main(int argc, char** argv){
  ros::init(argc, argv, "tffromscan");													  //initiation of this node, node id: tffromscan
 
  ros::NodeHandle node;																	  //state a node's handle
  ros::Subscriber sub = node.subscribe("/IMU", 10, &poseCallback);						  //subscribe to a topic named IMU

  ros::spin();																			  //enters a loop, calling message callbacks
  return 0;
};
