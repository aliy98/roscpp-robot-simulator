#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "std_srvs/Empty.h"

// for changing angle from rad to deg
#define RAD2DEG(x) ((x)*180./M_PI)

// defining some global variables
float d_th = 1.5;
float front_range = 2.0;
float right_range = 0.0;
float left_range = 0.0;
float linear_speed = 2.0;
bool reset_status = false;  

// callback function for controller node to read sensor data from /base_scan topic
void baseScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	front_range = scan->ranges[360]; // read front side range (at 90degree)

	for(int i = 0; i < 360; i++) right_range = right_range + scan->ranges[i];
	right_range = right_range/360; // compute the average of ranges from 0 to 90 degree

	for(int i = 0; i < 360; i++) left_range = left_range + scan->ranges[i+360];
	left_range = left_range/360;  // compute the average of ranges from 90 to 180 degree
}

// callback function for controller node to get commands from command node through /command topic
void commandCallback(const std_msgs::Float32::ConstPtr& msg)
{
	// if the command value is encoded as number three, set reset status as true
	if(msg->data == 3.0) reset_status = true; 
	//otherwise consider the command value as robot's linear speed
	else linear_speed = msg->data;
}


int main (int argc, char **argv)
{
	// initializing the controller node
	ros::init(argc, argv, "controller_node");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("/base_scan", 1,baseScanCallback); // to read sensor data 
	ros::Subscriber sub2 = nh.subscribe("/command", 1,commandCallback); // to get commands from command node
	ros::Publisher pub; 
	pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1); // to send control signal to the robot

	ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("/reset_positions"); // to request reset position service
	std_srvs::Empty reset_srv;
	

	ros::Rate loop_rate(10);
	while(ros::ok())
	{
		// request reset position service if the command node demands it
		if(reset_status == true)
		{
			client.call(reset_srv);
			reset_status = false;
		}

		// print sensor data and robot speed in terminal
		printf("front: %f\n",front_range);
		printf("right: %f\n",right_range);
		printf("left: %lf\n",left_range);
		printf("robot speed: %lf\n",linear_speed);
		printf("reset status: %d\n",reset_status);
		printf("\n");

		// implimentation of the control algorithm
		geometry_msgs::Twist my_vel;
		if(front_range > d_th) // if there is no obstacle in front of robot
		{
			// robot moves forwawrd
			my_vel.linear.x = linear_speed; 
			my_vel.angular.z = 0.0;

			// if robot is getting close to the obstacle from the left side
			if(left_range < 0.5*d_th)
			{
				// robot stops and turns to the right side
				my_vel.linear.x = 0.0;
				my_vel.angular.z = 2.0; 
			}

			// if robot is getting close to the obstacle from the right side
			if(right_range < 0.5*d_th)
			{
				// robot stops and turns to the left side
				my_vel.linear.x = 0.0;
				my_vel.angular.z = -2.0;
			}
		}
		else // there is an obstacle in front of robot
		{
			if(left_range > right_range) // if robot is closer to the obstacle in the left side
			{
				while(front_range < d_th & ros::ok()) 
				{
					// request reset position service if the command node demands it
					if(reset_status == true) 
					{
						client.call(reset_srv);
						reset_status = false;
					}

					// robot stops and turns to the right side until there is no obstacle in front of robot
					my_vel.linear.x = 0.0;
					my_vel.angular.z = 2.0;
					pub.publish(my_vel);

					// spin the loop
					ros::spinOnce();
					loop_rate.sleep();
				}
			}
			else  // if robot is closer to the obstacle in the right side
			{
				while(front_range < d_th & ros::ok())
				{
					// request reset position service if the command node demands it
					if(reset_status == true) 
					{
						client.call(reset_srv);
						reset_status = false;
					}

					// robot stops and turns to the left side until there is no obstacle in front of robot
					my_vel.linear.x = 0.0;
					my_vel.angular.z = -2.0;
					pub.publish(my_vel);

					// spin the loop
					ros::spinOnce();
					loop_rate.sleep();
				}
			}
		}

		//send the control signal to the robot
		pub.publish(my_vel);

		// spin the loop
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

