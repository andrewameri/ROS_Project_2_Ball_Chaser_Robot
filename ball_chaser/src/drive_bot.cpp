#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

//ROS::Publisher motor commands;
ros::Publisher motor_command_publisher;

// This callback function executes whenever a DriveToTarget service is requested
bool handle_drive_request(ball_chaser::DriveToTarget::Request& req,
		ball_chaser::DriveToTarget::Response& res)
{
	ROS_INFO("DrtiveToTargetRequest received - Linear_X:%1.2f, Angular_Z:%1.2f", (float)req.linear_x, (float)req.angular_z);
	
	//Degfining a geometry_msgs::Twist variable "motor_command" and assign corresponding values based on req
	geometry_msgs::Twist motor_command;
	motor_command.linear.x = (float)req.linear_x;
	motor_command.angular.z = (float)req.angular_z;
	
	//Publish motor_command 
	motor_command_publisher.publish(motor_command);
	
	//Return a response message
	res.msg_feedback = "Motor commands set - Linear_X: " + std::to_string((float)req.linear_x) + " , Angular_Z: " + std::to_string((float)req.angular_z);
	ROS_INFO_STREAM(res.msg_feedback);
	
	return true;
}

int main(int argc, char** argv)
{
	//Initialize a ROS node
	ros::init(argc, argv, "drive_bot");
	
	//Create a ROS NodeHandle object
	ros::NodeHandle n;
	
	//Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
	motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	
	//Define a drive /ball_chaser/command_robot service with a handle_drive_request callback function
	ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);
	ROS_INFO("Ready to send motor commands");

	// Handle ROS communication events
	ros::spin();
	
	return 0;
}