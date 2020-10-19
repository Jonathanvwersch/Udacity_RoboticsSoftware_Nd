#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
	ball_chaser::DriveToTarget srv;
	srv.request.linear_x = lin_x;
	srv.request.angular_z = ang_z;

	// Call the command_robot service and pass ang_z and lin_x
    if (!client.call(srv))
        ROS_ERROR("Failed to call service command_robot");


}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;
	int width = img.width;
	int height = img.height;
	int step = img.step;
	int left = 0;
	int right = 0;	
	int mid = 0;
	int position = 0;

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it

	for (int i = 0; i < step * height; i+=3) 
	{

		// If pixel is white
		if (img.data[i] == white_pixel && img.data[i+1] == white_pixel && img.data[i+3] == white_pixel)
		{
			position = i % img.step;

			// Pixel is in left of image
			if (position < step / 3) {
				++left;
			}
			
			// Pixel is in right of image
			else if (position >= step * 2/3) {
				++right;
			}

			// Pixel is in middle of image
			else {
				++mid;				
			}

		}
	}
	
	// Turn left
	if (left > mid && left > right) {
		ROS_INFO_STREAM("Turning left");
 		drive_robot(0, -0.5);
	}

	// Turn right
	else if (right > mid && right > left) {
		ROS_INFO_STREAM("Turning right");
		drive_robot(0.0, 0.5);
	}

	// Go straight
	else if (mid > left && mid > right) {
		ROS_INFO_STREAM("Going straight");
		drive_robot(0.5, 0.0);
	}

	// Stop
	else {
		ROS_INFO_STREAM("Not moving");
		drive_robot(0.0, 0.0);
	}
	
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


