#include <ros/ros.h>
//#include <tf/transform_broadcaster.h>
//#include <signal.h>
//#include <stdio.h>
//#include <string.h>
//#include <unistd.h>
//#include <fcntl.h> /* File control definitions */
//#include <errno.h>
//#include <termios.h> /* POSIX terminal control definitions */
//#include "sensor_msgs/Imu.h"
#include<visualization_msgs/Marker.h>

//int fd, odom_i;

//float x, y, x_f, y_f, z_f, p_f;

//int te,p_i;

//unsigned char p_c[4];
//unsigned char odom_c[4];


//int open_port(char** argv)
//{
//	fd = open(argv[1], O_RDWR | O_NOCTTY | O_NDELAY);
//	if (fd == -1)
//	{
//		/*
//		 * Could not open the port.
//		 */
//		perror("open_port: Unable to open /dev/ttyUSB0-");
//	}
//	else
//	{
//		return (fd);
//	}
//}


//float ByteToFloat(unsigned char* byteArry)
//{
//  return *((float*)byteArry);
//}

//int ByteToInt(unsigned char* byteArry)
//{
//  return *((int*)byteArry);
//}

//void tfpub(void)
//{
//  static tf::TransformBroadcaster br;
//  static tf::TransformBroadcaster br1;

//  tf::Transform transform;
//  tf::Transform transform1;


//  x += odom_i * cos(z_f/5730)/140000;
//  y += odom_i * sin(z_f/5730)/140000;
  
//  transform.setOrigin( tf::Vector3(x, y, 0) );                         //set the transform from father frame to child frame
//  transform1.setOrigin( tf::Vector3(0.1, 0, 0.35) );                         //set the transform from father frame to child frame
//  tf::Quaternion q;
//  tf::Quaternion q1;
//  q.setRPY(x_f/5730, y_f/5730, z_f/5730);				  //change the Euler angle to quaternion
//  q1.setRPY(0, -p_f/57.3, 0);
//  transform.setRotation(q);
//  transform1.setRotation(q1);
//  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));    //broadcast the tf message
//  br1.sendTransform(tf::StampedTransform(transform1, ros::Time::now(), "base_link", "laser"));    //broadcast the tf message
//}


//void poseCallback(const geometry_msgs::Twist::ConstPtr& msg){

//  x_f = msg->angular.x;

//  y_f = msg->angular.y;
//  z_f = msg->angular.z;				  //change the Euler angle to quaternion
//}


//int main(int argc, char** argv){
//  ros::init(argc, argv, "tanktf");					  //initiation of this node, node id: tffromscan
 

//  ros::NodeHandle node;								  //state a node's handle

//  ros::Subscriber sub = node.subscribe("/IMU", 10, &poseCallback);
 
//  ros::Rate r(50);

//  fd = open_port(argv);

//  while(1)
//  {
//    write(fd, "0", 1);
//    ros::spinOnce();
//    r.sleep();

//    te=read(fd, odom_c, 4);
//    odom_i=ByteToInt(odom_c);
//    printf("%d\n ", odom_i);
////    printf("%d\n", te);

//    te=read(fd, p_c, 4);
//    p_f=ByteToFloat(p_c);
//    p_i = p_f;
//    printf("%d, ", p_i);
////    printf("%f\n", y_f);
////    printf("%d\n", te);

//    tfpub();



//  }

//									  //enters a loop, calling message callbacks
//  return 0;
//};



int main(int argc, char** argv )
{
    ros::init(argc, argv, "maker");


    ros::NodeHandle node_handle;
    ros::Rate r(10);
    ros::Publisher vis_pub = node_handle.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 2.25;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0.25;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;


    visualization_msgs::Marker txt;
    txt.header.frame_id = "map";
    txt.header.stamp = ros::Time();
    txt.ns = "my_namespace";
    txt.id = 1;
    txt.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    txt.action = visualization_msgs::Marker::ADD;
    txt.pose.position.x = 0.0;
    txt.pose.position.y = -0.0;
    txt.pose.position.z = -0.1;
    txt.pose.orientation.x = 0.0;
    txt.pose.orientation.y = 0.0;
    txt.pose.orientation.z = 0.0;
    txt.pose.orientation.w = 1.0;
//    txt.scale.x = 0.5;
//    txt.scale.y = 0.5;
    txt.scale.z = 0.1;
    txt.color.a = 1.0; // Don't forget to set the alpha!
    txt.color.r = 0.5;
    txt.color.g = 0.5;
    txt.color.b = 0.5;
    txt.text="(0,0,0)";
    //only if using a MESH_RESOURCE marker type:
  //  marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
    while(1)
    {
        vis_pub.publish( marker );
        vis_pub.publish( txt );
        r.sleep();
        ros::spinOnce();
    }
   //

}

