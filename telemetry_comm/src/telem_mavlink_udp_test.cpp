/*******************************************************************************
 Copyright (C) 2017  mohamedashraf123 ( a t ) gmail.com
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 
 ****************************************************************************/
/* These headers are for QNX, but should all be standard on unix/linux */
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <time.h>
#if (defined __QNX__) | (defined __QNXNTO__)
/* QNX specific headers */
#include <unix.h>
#else
/* Linux / MacOS POSIX timer headers */
#include <sys/time.h>
#include <time.h>
#include <arpa/inet.h>
#endif

/* This assumes you have the mavlink headers on your include path
 or in the same folder as this source file */
#include <mavlink/v1.0/ch3_custom/mavlink.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"
#include "autopilots/StateMachine.h"

#define BUFFER_LENGTH 2041 // minimum buffer size that can be used with qnx (I don't know why)

uint64_t microsSinceEpoch();

using namespace std;

/* Listens to user command on '/gripper_command' topic. Once available, sends it to gripper's microcontroller */
class MyCallbacks
{
public:
     MyCallbacks() { }
    ~MyCallbacks() { }

    autopilots::StateMachine _sm_msg;
    sensor_msgs::NavSatFix _gps_msg;

    // callback
    //gps
    void gps_cb(const sensor_msgs::NavSatFix::ConstPtr& msg)
    {
        _gps_msg.header = msg->header;
        _gps_msg.latitude = msg->latitude;
        _gps_msg.longitude = msg->longitude;
    }
    //state
    void sm_cb(const autopilots::StateMachine::ConstPtr& msg)
    {
        _sm_msg.header = msg->header;
        _sm_msg.state = msg->state;
    }

};

int main(int argc, char **argv)
{

	/**
	make sure to get port/baud from command line arguments
	*/
	if (argc != 4)
	{
	ROS_INFO("telem_serial quadN /dev/ttyACM0 57600");
	ROS_ERROR("Please provide drone ID");
		return 1;
	}
	// my drone number
	uint8_t droneN=(uint8_t)(atoi(argv[1]));



	/**
	* You must call one of the versions of ros::init() before using any other
	* part of the ROS system.
	*/
	ros::init(argc, argv, "telem_mavlink_udp");

	/**
	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/
	ros::NodeHandle n("~");

	//Get the namespace
	std::string ns = ros::this_node::getNamespace();

	ros::Rate loop_rate(20);	/* Hz */

	// other quads numbers
	uint8_t droneA_N = 2;
	uint8_t droneB_N = 3;
	// other quads name spaces
	string droneA_ns = "/Quad2";
	string droneB_ns = "/Quad3";

	// resolve other quads numbers
	if (droneN == 1)
	{
	droneA_N = 2;
	droneB_N = 3;
	droneA_ns = "/Quad2";
	droneB_ns = "/Quad3";
	}
	else if(droneN == 2){
	droneA_N = 1;
	droneB_N = 3;
	droneA_ns = "/Quad1";
	droneB_ns = "/Quad3";

	}
	else if(droneN == 3){
	droneA_N = 1;
	droneB_N = 2;
	droneA_ns = "/Quad1";
	droneB_ns = "/Quad2";

	}
	else{
	ROS_ERROR("Drone ID is not 1, 2, or 3");
	return 1;
	}

	// create other quads msg to be published
	sensor_msgs::NavSatFix droneA_gps_msg;
	autopilots::StateMachine droneA_sm_msg;

	sensor_msgs::NavSatFix droneB_gps_msg;
	autopilots::StateMachine droneB_sm_msg;

	/* other quads msg publisher */
	ros::Publisher droneA_gps_pub = n.advertise<sensor_msgs::NavSatFix>(droneA_ns+"/mavros/global_position/global", 10);
	ros::Publisher droneB_gps_pub = n.advertise<sensor_msgs::NavSatFix>(droneB_ns+"/mavros/global_position/global", 10);

	ros::Publisher droneA_sm_pub = n.advertise<autopilots::StateMachine>(droneA_ns+"/state_machine/state", 10);
	ros::Publisher droneB_sm_pub = n.advertise<autopilots::StateMachine>(droneB_ns+"/state_machine/state", 10);

	/* Subscribers to this quads gps and state*/
	MyCallbacks my_cb;			/* holds the callbacks for this quad's gps and state*/
	ros::Subscriber my_gps_sub = n.subscribe(ns+"/mavros/global_position/global", 10, &MyCallbacks::gps_cb, &my_cb);
	ros::Subscriber my_sm_sub = n.subscribe(ns+"/state_machine/state", 10, &MyCallbacks::sm_cb, &my_cb);

	int sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
	struct sockaddr_in droneA_Addr;
	struct sockaddr_in droneB_Addr;  
	struct sockaddr_in locAddr;
	//struct sockaddr_in fromAddr;
	uint8_t buf[BUFFER_LENGTH];// send buffer
	uint8_t rcv_buf1[BUFFER_LENGTH];
	uint8_t rcv_buf2[BUFFER_LENGTH];
	ssize_t recsize1, recsize2;
	socklen_t fromlen;
	int bytes_sent;
	mavlink_message_t msg;
	uint16_t len;
	int i = 0;
	bool droneA_updated = false;
	bool droneB_updated = false;

	// define ips and ports
	std::string d1_ip, d2_ip, d3_ip;
	int d1_prt, d2_prt, d3_prt;
	char droneA_ip[100];
	char droneB_ip[100];
	int myport = 5005;
	int droneA_port= 5006;
	int droneB_port= 5007;
	
	// TODO read quad ips from ros paramters
	if (!ros::param::get("/drone1_ip", d1_ip)){
		ROS_ERROR("drone1 ip is not loaded");
		return 1;
	}
	if (!ros::param::get("/drone2_ip", d2_ip)){
		ROS_ERROR("drone2 ip is not loaded");
		return 1;
	}
	if (!ros::param::get("/drone3_ip", d3_ip)){
		ROS_ERROR("drone3 ip is not loaded");
		return 1;
	}

	// TODO read drones' udp ports from ros paramters
	if (!ros::param::get("/drone1_port", d1_prt)){
		ROS_ERROR("drone1 port is not loaded");
		return 1;
	}
	if (!ros::param::get("/drone2_port", d2_prt)){
		ROS_ERROR("drone2 port is not loaded");
		return 1;
	}
	if (!ros::param::get("/drone3_port", d3_prt)){
		ROS_ERROR("drone3 port is not loaded");
		return 1;
	}

	// set my ip/port, and other drones
	switch (droneN){
		case 1:
			strcpy(droneA_ip, d2_ip.c_str());
			strcpy(droneB_ip, d3_ip.c_str());
			myport = d1_prt;
			droneA_port = d2_prt;
			droneB_port = d3_prt;
			break;
		case 2:
			strcpy(droneA_ip, d1_ip.c_str());
			strcpy(droneB_ip, d3_ip.c_str());
			myport = d2_prt;
			droneA_port = d1_prt;
			droneB_port = d3_prt;
			break;
		case 3:
			strcpy(droneA_ip, d1_ip.c_str());
			strcpy(droneB_ip, d2_ip.c_str());
			myport = d3_prt;
			droneA_port = d1_prt;
			droneB_port = d2_prt;
			break;
		default:
			ROS_ERROR("Drone number is > 3");
			return 1;
			break;
	}
	// set local address object
	memset(&locAddr, 0, sizeof(locAddr));
	locAddr.sin_family = AF_INET;
	locAddr.sin_addr.s_addr = INADDR_ANY;
	locAddr.sin_port = htons(myport);

	// other drones addresses
	memset(&droneA_Addr, 0, sizeof(droneA_Addr));
	droneA_Addr.sin_family = AF_INET;
	droneA_Addr.sin_addr.s_addr = inet_addr(droneA_ip);
	droneA_Addr.sin_port = htons(droneA_port);

	memset(&droneB_Addr, 0, sizeof(droneB_Addr));
	droneB_Addr.sin_family = AF_INET;
	droneB_Addr.sin_addr.s_addr = inet_addr(droneB_ip);
	droneB_Addr.sin_port = htons(droneB_port);

	/* Bind to local port - necessary to receive packets from other drones */ 
	if (-1 == bind(sock,(struct sockaddr *)&locAddr, sizeof(struct sockaddr)))
	{
		perror("error bind failed");
		close(sock);
		exit(EXIT_FAILURE);
	} 
	
	/* Attempt to make it non blocking */
	if (fcntl(sock, F_SETFL, O_NONBLOCK | FASYNC) < 0)
	{
		fprintf(stderr, "error setting nonblocking: %s\n", strerror(errno));
		close(sock);
		exit(EXIT_FAILURE);
	}

	/* ROS loop */
	while (ros::ok())
	{

		/* fill in test msg & send */
		uint8_t mysID=5;

		// fill msg
		mavlink_msg_state_machine_status_pack(1, 200, &msg, microsSinceEpoch(), droneN, mysID, 23.123456, 8.123456);

		// fill buffer and send
		memset(buf, 0, BUFFER_LENGTH);
		len = mavlink_msg_to_send_buffer(buf, &msg);
		bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&droneA_Addr, sizeof(struct sockaddr_in));
		sleep(0.01);
		bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&droneB_Addr, sizeof(struct sockaddr_in)); 

		/* recieve */
		memset(rcv_buf1, 0, BUFFER_LENGTH);
		memset(rcv_buf2, 0, BUFFER_LENGTH);
		// recieve from drone A
		recsize1 = recvfrom(sock, (void *)rcv_buf1, BUFFER_LENGTH, 0, (struct sockaddr *)&droneA_Addr, &fromlen);
		// recieve from drone B 
		recsize2 = recvfrom(sock, (void *)rcv_buf2, BUFFER_LENGTH, 0, (struct sockaddr *)&droneB_Addr, &fromlen);

		//parse 1
		if (recsize1 > 0){
			// Something received - print out all bytes and parse packet
			mavlink_message_t msg;
			mavlink_status_t status;
	
			for (i = 0; i < recsize1; ++i)
			{
				if (mavlink_parse_char(MAVLINK_COMM_0, rcv_buf1[i], &msg, &status))
				{
					//chek msg id
					switch (msg.msgid){
						case MAVLINK_MSG_ID_STATE_MACHINE_STATUS:
							mavlink_state_machine_status_t state_msg;
							mavlink_msg_state_machine_status_decode(&msg, &state_msg);
							std::string st = "Unknown";
							switch (state_msg.state){
							case 0:
								st = "Start";
								break;
							case 1:
								st = "Takeoff";
								break;
							case 2:
								st = "ObjectSearch";
								break;
							case 3:
								st = "Picking";
								break;
							case 4:
								st = "GoToDrop";
								break;
							case 5:
								st = "WaitToDrop";
								break;
							case 6:
								st = "Drop";
								break;
							case 7:
								st = "Hover";
								break;
							default:
								st = "Unknown";
								break;
							}		
							// check if there is duplicate drone id
							if (state_msg.drone_id == droneN){
								ROS_ERROR("Duplicated drone ID");
								return 1;
							}
							if (state_msg.drone_id == droneA_N){
								droneA_updated = true;
								droneA_gps_msg.header.stamp = ros::Time::now();
								droneA_gps_msg.latitude = state_msg.latitude;
								droneA_gps_msg.longitude = state_msg.longitude;

								droneA_sm_msg.header.stamp = ros::Time::now();
								droneA_sm_msg.state = st;
							}
							if (state_msg.drone_id == droneB_N){
								droneB_updated = true;
								droneB_gps_msg.header.stamp = ros::Time::now();
								droneB_gps_msg.latitude = state_msg.latitude;
								droneB_gps_msg.longitude = state_msg.longitude;

								droneB_sm_msg.header.stamp = ros::Time::now();
								droneB_sm_msg.state = st;
							}
							break;
					}// end of switch msgid
				}// end parsing mavlink id
			}
		}// end of rcv_buff1

		//parse 2
		if (recsize2 > 0){
			// Something received - print out all bytes and parse packet
			mavlink_message_t msg;
			mavlink_status_t status;
	
			for (i = 0; i < recsize1; ++i)
			{
				if (mavlink_parse_char(MAVLINK_COMM_0, rcv_buf2[i], &msg, &status))
				{
					//chek msg id
					switch (msg.msgid){
						case MAVLINK_MSG_ID_STATE_MACHINE_STATUS:
							mavlink_state_machine_status_t state_msg;
							mavlink_msg_state_machine_status_decode(&msg, &state_msg);
							std::string st = "Unknown";
							switch (state_msg.state){
							case 0:
								st = "Start";
								break;
							case 1:
								st = "Takeoff";
								break;
							case 2:
								st = "ObjectSearch";
								break;
							case 3:
								st = "Picking";
								break;
							case 4:
								st = "GoToDrop";
								break;
							case 5:
								st = "WaitToDrop";
								break;
							case 6:
								st = "Drop";
								break;
							case 7:
								st = "Hover";
								break;
							default:
								st = "Unknown";
								break;
							}		
							// check if there is duplicate drone id
							if (state_msg.drone_id == droneN){
								ROS_ERROR("Duplicated drone ID");
								return 1;
							}
							if (state_msg.drone_id == droneA_N){
								droneA_updated = true;
								droneA_gps_msg.header.stamp = ros::Time::now();
								droneA_gps_msg.latitude = state_msg.latitude;
								droneA_gps_msg.longitude = state_msg.longitude;

								droneA_sm_msg.header.stamp = ros::Time::now();
								droneA_sm_msg.state = st;
							}
							if (state_msg.drone_id == droneB_N){
								droneB_updated = true;
								droneB_gps_msg.header.stamp = ros::Time::now();
								droneB_gps_msg.latitude = state_msg.latitude;
								droneB_gps_msg.longitude = state_msg.longitude;

								droneB_sm_msg.header.stamp = ros::Time::now();
								droneB_sm_msg.state = st;
							}
							break;
					}
				}
			}
		}// end of rcv_buff2

		/* ------------ publish msg ----------- */
		//Drone A
		if (droneA_updated){
			droneA_gps_pub.publish(droneA_gps_msg);
			droneA_sm_pub.publish(droneA_sm_msg);
			droneB_updated = false;
		}

		if(droneB_updated){
			droneB_gps_pub.publish(droneB_gps_msg);
			droneB_sm_pub.publish(droneB_sm_msg);
			droneB_updated = false;
		}

		ros::spinOnce();
	        loop_rate.sleep();

	} // end of ROS loop

	
}

/* QNX timer version */
#if (defined __QNX__) | (defined __QNXNTO__)
uint64_t microsSinceEpoch()
{
	
	struct timespec time;
	
	uint64_t micros = 0;
	
	clock_gettime(CLOCK_REALTIME, &time);  
	micros = (uint64_t)time.tv_sec * 1000000 + time.tv_nsec/1000;
	
	return micros;
}
#else
uint64_t microsSinceEpoch()
{
	
	struct timeval tv;
	
	uint64_t micros = 0;
	
	gettimeofday(&tv, NULL);  
	micros =  ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;
	
	return micros;
}
#endif
