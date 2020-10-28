#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <stdlib.h>
#include "std_msgs/String.h"

geometry_msgs::Pose target;
geometry_msgs::Pose odom;
std_msgs::String arrival_msg;

void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	 odom.position.x = msg->pose.pose.position.x;
	 odom.position.y = msg->pose.pose.position.y;
	 odom.position.z = msg->pose.pose.position.z;
   odom.orientation.x = msg->pose.pose.orientation.x;
   odom.orientation.y = msg->pose.pose.orientation.y;
   odom.orientation.z = msg->pose.pose.orientation.z;
   odom.orientation.w = msg->pose.pose.orientation.w;
}

void target_callback(const geometry_msgs::Pose &msg)
{
	target.position.x = msg.position.x;
	target.position.y = msg.position.y;
	target.position.z = msg.position.z;
	target.orientation.x = msg.orientation.x;
	target.orientation.y = msg.orientation.y;
	target.orientation.z = msg.orientation.z;
	target.orientation.w = msg.orientation.w;
}

void msg_callback(const std_msgs::String::ConstPtr& msg)
{
	arrival_msg.data = msg->data.c_str();

}

int main( int argc, char** argv )
{
		ros::init(argc, argv, "add_markers");
		ros::NodeHandle n;
		ros::Rate r(1);
		
		ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
		ros::Subscriber target_sub = n.subscribe("target", 10, target_callback);
		ros::Subscriber odom_sub = n.subscribe("odom", 10, odom_callback);
		ros::Subscriber msg_sub = n.subscribe("msg", 10, msg_callback);
		
		int cycle = 0;
		float error = 0.1;
		bool isOnWayToDropOff = false;

		float firstTarget = abs(target.position.x);

		uint32_t shape = visualization_msgs::Marker::CUBE;

		while(ros::ok())
		{

			visualization_msgs::Marker marker;

			// Set the frame ID and timestamp.  See the TF tutorials for information on these.
			marker.header.frame_id = "/map";
			marker.header.stamp = ros::Time::now();

			// Set the namespace and id for this marker.  This serves to create a unique ID
			// Any marker sent with the same namespace and id will overwrite the old one
			marker.ns = "add_markers";
			marker.id = 0;

			// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
			marker.type = shape;
	

			// Set the initial pose of the marker.
			marker.pose.position.x = target.position.x;
			marker.pose.position.y = target.position.y;
			marker.pose.position.z = target.position.z;
			marker.pose.orientation.x = target.orientation.x;
			marker.pose.orientation.y = target.orientation.y;
			marker.pose.orientation.z = target.orientation.z;
			marker.pose.orientation.w = target.orientation.w;

			// Set the scale of the marker 
			marker.scale.x = 0.5;
			marker.scale.y = 0.5;
			marker.scale.z = 0.5;

			// Set the color -- be sure to set alpha to something non-zero!
			marker.color.r = 0.0f;
			marker.color.g = 1.0f;
			marker.color.b = 0.0f;
			marker.color.a = 1.0;

			marker.lifetime = ros::Duration();
			
			ROS_INFO("%f, %f, %f, %f",target.position.x, target.position.y, odom.position.x, odom.position.y);
			ROS_INFO("%s", arrival_msg.data.c_str());

			if (abs(odom.position.x - target.position.x) < error && abs(odom.position.y - target.position.y) < error && arrival_msg.data == "pickup")
			{
				marker.action = visualization_msgs::Marker::DELETE;
				ROS_INFO("Package picked up");
				marker_pub.publish(marker);
				cycle += 1;
			}


			else if (abs(odom.position.x - target.position.x) < error && abs(odom.position.y - target.position.y) < error && arrival_msg.data == "dropoff")
			{
				marker.action = visualization_msgs::Marker::ADD;
				ROS_INFO("Package dropped off");
				marker_pub.publish(marker);

			}

			else if (cycle == 0) 
			{
				marker.action = visualization_msgs::Marker::ADD;
				marker_pub.publish(marker);
			}
							

			ros::spinOnce();
			r.sleep();
		}


}
