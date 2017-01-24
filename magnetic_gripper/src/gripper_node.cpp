#include "ros/ros.h"
#include "std_msgs/String.h"
#include "serial/serial.h"
#include "autopilots/GripperAction.h"
#include "autopilots/GripperFeedback.h"

#include <string>

/* Header bytes for feedback/command messages */
#define FEEDBACK_HEADER 7
#define COMMAND_HEADER 9

using namespace std;

/* Listens to user command on '/gripper_command' topic. Once available, sends it to gripper's microcontroller */
class ActionListner
{
public:
	 ActionListner() { }
	~ActionListner() { }

	autopilots::GripperAction _g_action;
	serial::Serial* _serial_ptr;

	// callback
	void cb(const autopilots::GripperAction::ConstPtr& msg)
	{
		_g_action.command = msg->command;
		ROS_INFO("I heard: [%d]", _g_action.command);

		if(_serial_ptr->isOpen())
			cout << " Yes." << endl;
		else
			cout << " No." << endl;
	}

	
};
/*--------------------------------------------------------*/


/** MAIN:
* This node interacts with the magnetic gripper controller, Aurduino.
* 1) opens a serial port to talk to aurduino
* 2) listens to gripper status, and updates/publishes the corresponding topic
* 3) listens to the commands topics (by user),  and sends commands to aurduino
*/
int main(int argc, char **argv)
{
	/**
	* You must call one of the versions of ros::init() before using any other
	* part of the ROS system.
	*/
	ros::init(argc, argv, "gripper_node");

	/**
	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/
	ros::NodeHandle n("~");

	ros::Rate loop_rate(10);	/* Hz */

	/* Serial setup */
	uint8_t *buffer;		/* read buffer */
	size_t bytes_read;		/* number of bytes read from serail port returned by read() method*/
	size_t bytes_available;		/* bytes available, returned by available() method*/
	string port;
	n.param<std::string>("address", port, "/dev/ttyACM0");
	int baud;
	n.param("baud", baud, 57600);
	/* create serial handle */
	serial::Serial serial_handle(port, (unsigned long)baud, serial::Timeout::simpleTimeout(1000));
	ROS_INFO("Serial port is opened");

	/* Gripper's topics*/
	autopilots::GripperFeedback g_status;
	autopilots::GripperAction g_action;

	/* Gripper state publisher */ 
	ros::Publisher gripper_pub = n.advertise<autopilots::GripperFeedback>("gripper_status", 10);

	/* Gripper subscriber: subscribes to user commands, and send them to gripper microcontroller*/
	ActionListner cmd_listner;			/* holds the callback and deos the serial write to microcontroller*/
	cmd_listner._serial_ptr = &serial_handle;	/* pointer to the serial handle. Used inside the class object to write commands to same serial port */
	ros::Subscriber cmd_sub = n.subscribe("gripper_command", 10, &ActionListner::cb, &cmd_listner);

	/* Main loop */
	while (ros::ok())
	{

		if (serial_handle.isOpen())
		{
			/* read serial buffer, if data is available */
			bytes_available = serial_handle.available();
			ROS_INFO("D: %d\n", bytes_available);
			if ( bytes_available > 0)
			{
				bytes_read = serial_handle.read(buffer, serial_handle.available());
			}

			/* parse data */
			if (bytes_read > 0 )
			{
				/* loop over the whole buffer */
				for (int i=0; i < bytes_read; i++)
				{
					/* two bytes are expected which represent: (msg header)| (a byte for the state) */
					if ((uint8_t)buffer[i] == FEEDBACK_HEADER && (i+1) < bytes_read) /* make sure we always can read 2 bytes. Otherwise, wrong msg */
					{
						if ( (uint8_t)buffer[i+1] == 1)
							g_status.picked = true;
						else
							g_status.picked = false;
						i++;
					}
				} /* finished reading the buffer */
			} /* finished parsing data */
		}
		else
		{
			ROS_INFO("Serial port is not open. Shutting down the node.");
			break;
		}

		ROS_INFO("%d\n", g_status.picked);

		/* publish the gripper status */
		gripper_pub.publish(g_status);

		ros::spinOnce();

		loop_rate.sleep();
	}

	/* close port on exit */
	ROS_INFO("Closing serial port\n");
	serial_handle.close();


	return 0;
}
