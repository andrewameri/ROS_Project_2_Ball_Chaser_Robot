#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

//Define global client that can request services
ros::ServiceClient client;

//This function calls the command_robot service to drive the robot
void drive_robot(float lin_x, float ang_z)
{
	//Request a service and pass velocities to drive the robot
	ROS_INFO_STREAM("Moving the robot");
	
	ball_chaser::DriveToTarget srv;
	srv.request.linear_x = lin_x;
	srv.request.angular_z = ang_z;
	
	// Call the safe_move service and pass the requested joint angles
	if (!client.call(srv))
		ROS_ERROR("Failed to call service safe_move");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    int white_pixel = 255;
    bool ball_found = false;
    int ball_col;
    float robot_lin_x = 0;
    float robot_ang_z = 0;
    
    for (int i=0; i<img.height * img.step; i+=3)
    {
    	if(img.data[i]==white_pixel && img.data[i+1]==white_pixel && img.data[i+2]==white_pixel)
    	{
    		ball_found = true;
    		ball_col = (int)((i/3) % img.width);
    	}
    }
    
    if(ball_found)
    {
    	if(ball_col < (img.width/3))
    		drive_robot(0.1, 0.5);
    	else if(ball_col >= (img.width/3) && ball_col < (2*img.width/3))
    		drive_robot(0.5, 0);
    	else
    		drive_robot(0.1, -0.5);
    }
    
    else
    	drive_robot(0, 0);
}

int main(int argc, char** argv)
{
	// Initialize the process_image node and create a handle to it
	ros::init(argc, argv, "process_image");
	ros::NodeHandle n;
	
	// Define a client service capable of requesting services from command_robot
	client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");
	
	// Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
	ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);
	
	// Handle ROS communication events
	ros::spin();
	
	return 0;
}