#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/String.h"

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");
  
  // Create node handle--ros's interface for creating publishers and subscribers
  ros::NodeHandle n;
  
  // Publish to target topic
  ros::Publisher goal_pub = n.advertise<geometry_msgs::Pose>("target", 1000);
	ros::Publisher goal_pub_msg = n.advertise<std_msgs::String>("msg", 1000);

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

/* Define Pickup Location Parameters */

  move_base_msgs::MoveBaseGoal pickup_goal;

  // set up the frame parameters
  pickup_goal.target_pose.header.frame_id = "/map";
  pickup_goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach; this is its pickup goal
  pickup_goal.target_pose.pose.position.x = -2.5;
  pickup_goal.target_pose.pose.position.y = 1.5;
  pickup_goal.target_pose.pose.orientation.w = 1.0;
  
  while(goal_pub.getNumSubscribers() < 1)
  {
  	if (!ros::ok())
  		return 0;
  		
  	ROS_WARN("add_markers has not subscribed to target topic");
  	sleep(5);
  }

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending robot to pickup location");
  ac.sendGoal(pickup_goal);

  
  geometry_msgs::Pose pickup_msg;
	std_msgs::String arrival_msg;
  
  pickup_msg.position.x = -2.5;
  pickup_msg.position.y = 1.5;
  pickup_msg.position.z = 0;
  pickup_msg.orientation.x = 0.0;
  pickup_msg.orientation.y = 0.0;
  pickup_msg.orientation.z = 0.0;
  pickup_msg.orientation.w = 1.0;
  goal_pub.publish(pickup_msg);
  
  ROS_INFO("Pick up location published to topic, x = %f, y = %f, w = %f",
  	pickup_msg.position.x,
  	pickup_msg.position.y,
  	pickup_msg.orientation.w
  );
  
    // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached pickup location
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Hooray, the robot arrived at its pickup location");
		arrival_msg.data = "pickup";
		goal_pub_msg.publish(arrival_msg);
    ros::Duration(5.0).sleep();
		
  }
    
  else
  {
    ROS_INFO("The robot failed to arrive at its pickup location");
    ros::Duration(5.0).sleep();
  }
 

/* Define Drop-off Location Parameters */

  move_base_msgs::MoveBaseGoal dropoff_goal;
  
  // Set up frame parameters
  dropoff_goal.target_pose.header.frame_id = "map";
  dropoff_goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach; this its drop-off location
  dropoff_goal.target_pose.pose.position.x = -3.8;
  dropoff_goal.target_pose.pose.position.y = -3.5;
  dropoff_goal.target_pose.pose.orientation.w = 1.0;

  // Send the dropoff location and orientation 
  ROS_INFO("Sending robot to dropoff location");
  ac.sendGoal(dropoff_goal);
  
  geometry_msgs::Pose dropoff_msg;
  
  dropoff_msg.position.x = -3.8;
  dropoff_msg.position.y = -3.5;
  dropoff_msg.position.z = 0;
  dropoff_msg.orientation.x = 0.0;
  dropoff_msg.orientation.y = 0.0;
  dropoff_msg.orientation.z = 0.0;
  dropoff_msg.orientation.w = 1.0;
	ros::Duration(7.0).sleep();
	goal_pub.publish(dropoff_msg);
 
  
  ROS_INFO("Dropoff location published to topic, x = %f, y = %f, w = %f",
  	dropoff_msg.position.x,
  	dropoff_msg.position.y,
  	dropoff_msg.orientation.w
  );
  

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached pickup location
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Hooray, the robot arrived at its dropoff location");
		arrival_msg.data = "dropoff";
		goal_pub_msg.publish(arrival_msg);
    ros::Duration(5.0).sleep();
		
  }
    
  else
  {
    ROS_INFO("The robot failed to arrive at its dropoff location");
    ros::Duration(5.0).sleep();
  }
  
  return 0;
}
