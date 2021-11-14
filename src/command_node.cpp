#include "ros/ros.h"
#include "std_msgs/Float32.h"

int main (int argc, char **argv)
{
	// initializing the command node
	ros::init(argc, argv, "command_node");  
	ros::NodeHandle nh;
    	ros::Publisher pub;
	pub = nh.advertise<std_msgs::Float32>("/command",1);  //to send commands to the controller node
	char buffer[80];

	ros::Rate loop_rate(10);
	while(ros::ok())
	{
		std_msgs::Float32 msg;
		printf("enter robot speed or enter \"r\" to reset robot position:  "); // get command from user
        scanf("%s",buffer);

		// if user wants to reset the position of robot
        if(buffer[0] == 'r') 
		{
			// encode the reset command as number three so the data can be sent as a float32 std_msg
			msg.data = 3.0;
			pub.publish(msg); // send command to controller node
		}

		// if user wants to change robot's speed
        else 
        {
            // change the input string to float so it can be sent as a float32 std_msg
			msg.data = std::stof(buffer);
			// check if the input speed is accaptable
            if(msg.data > 2.0) printf("error: max value is 2\n"); 
			else if((msg.data < 0.0)) printf("error: min value is 0\n");
            else pub.publish(msg); // send command to controller node
        }
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

