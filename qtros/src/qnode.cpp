/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../include/qtros/qnode.hpp"
#include "ui_main_window.h"
#include "../include/qtros/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qtros {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"qtros");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
// //	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);

  chatter_subscriber = n.subscribe("turtle1/cmd_vel", 1000, &QNode::chatterCallback,this);
  chatter_subscriber2=n.subscribe("turtle2/cmd_vel",1000, &QNode::chatterCallback2,this);
  AltitudeSubscriber1=n.subscribe("Quad1/mavros/altitude",1000, &QNode::AltitudeCallback1,this);
  AltitudeSubscriber2=n.subscribe("Quad2/mavros/altitude",1000, &QNode::AltitudeCallback2,this);
  AltitudeSubscriber3=n.subscribe("Quad3/mavros/altitude",1000, &QNode::AltitudeCallback3,this);
  BatterySubscriber1=n.subscribe("Quad1/mavros/battery",1000, &QNode::BatteryCallback1,this);
  BatterySubscriber2=n.subscribe("Quad2/mavros/battery",1000, &QNode::BatteryCallback2,this);
  BatterySubscriber3=n.subscribe("Quad3/mavros/battery",1000, &QNode::BatteryCallback3,this);
  PositionSubscriber1=n.subscribe("Quad1/mavros/local_position/pose",1000,&QNode::PositionCallback1,this);
  PositionSubscriber2=n.subscribe("Quad2/mavros/local_position/pose",1000,&QNode::PositionCallback2,this);
  PositionSubscriber3=n.subscribe("Quad3/mavros/local_position/pose",1000,&QNode::PositionCallback3,this);
  VelocitySubscriber1=n.subscribe("Quad1/mavros/local_position/velocity",1000,&QNode::VelocityCallback1,this);
  VelocitySubscriber2=n.subscribe("Quad2/mavros/local_position/velocity",1000,&QNode::VelocityCallback2,this);
  VelocitySubscriber3=n.subscribe("Quad3/mavros/local_position/velocity",1000,&QNode::VelocityCallback3,this);
  StateMachineSubscriber1=n.subscribe("/Quad1/state_machine/state",1000,&QNode::StateMachineCallback1,this);
  StateMachineSubscriber2=n.subscribe("/Quad2/state_machine/state",1000,&QNode::StateMachineCallback2,this);
  StateMachineSubscriber3=n.subscribe("/Quad3/state_machine/state",1000,&QNode::StateMachineCallback3,this);
  QuadStateSubscriber1=n.subscribe("/Quad1/mavros/state",1000,&QNode::QuadStateCallback1,this);
  QuadStateSubscriber2=n.subscribe("/Quad2/mavros/state",1000,&QNode::QuadStateCallback2,this);
  QuadStateSubscriber3=n.subscribe("/Quad3/mavros/state",1000,&QNode::QuadStateCallback3,this);
	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"qtros");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
	start();
	return true;
}

void QNode::chatterCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
//  std::string buffer;
//  buffer=boost::lexical_cast<std::string>(msg->linear.x);
//  ROS_INFO("I heard: [%f] and [%f]", msg->linear.x,msg->angular.z);
//  log(Info,std::string("I sent: ")+buffer);
  Q_EMIT CallBackTrigger(msg->linear.x, msg->angular.z);
}
void QNode::chatterCallback2(const geometry_msgs::Twist::ConstPtr &msg)
{
  Q_EMIT CallBackTrigger2(msg->linear.x,msg->angular.z);
}
void QNode::AltitudeCallback1(const mavros_msgs::Altitude::ConstPtr& msg)
{
  Q_EMIT AltitudeSignal1(msg->local);
}
void QNode::AltitudeCallback2(const mavros_msgs::Altitude::ConstPtr& msg)
{
  Q_EMIT AltitudeSignal2(msg->local);
}
void QNode::AltitudeCallback3(const mavros_msgs::Altitude::ConstPtr& msg)
{
  Q_EMIT AltitudeSignal3(msg->local);
}
void QNode::BatteryCallback1(const mavros_msgs::BatteryStatus::ConstPtr &msg) // previous: sensor_msgs/BatteryStates
{
  Q_EMIT BatterySignal1(msg->remaining,msg->voltage);       // previous: msg->percentage
}
void QNode::BatteryCallback2(const mavros_msgs::BatteryStatus::ConstPtr &msg)
{
  Q_EMIT BatterySignal2(msg->remaining,msg->voltage);
}
void QNode::BatteryCallback3(const mavros_msgs::BatteryStatus::ConstPtr &msg)
{
  Q_EMIT BatterySignal3(msg->remaining,msg->voltage);
}
void QNode::PositionCallback1(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  Q_EMIT PositionSignal1(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
}
void QNode::PositionCallback2(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  Q_EMIT PositionSignal2(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
}
void QNode::PositionCallback3(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  Q_EMIT PositionSignal3(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
}
void QNode::VelocityCallback1(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
  Q_EMIT VelocitySignal1(msg->twist.linear.x,msg->twist.linear.y,msg->twist.linear.z);
}
void QNode::VelocityCallback2(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
  Q_EMIT VelocitySignal2(msg->twist.linear.x,msg->twist.linear.y,msg->twist.linear.z);
}
void QNode::VelocityCallback3(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
  Q_EMIT VelocitySignal3(msg->twist.linear.x,msg->twist.linear.y,msg->twist.linear.z);
}
void QNode::StateMachineCallback1(const autopilots::StateMachine::ConstPtr &msg)
{
  Q_EMIT StateMachineSignal1(msg->state,msg->signal);
}
void QNode::StateMachineCallback2(const autopilots::StateMachine::ConstPtr &msg)
{
  Q_EMIT StateMachineSignal2(msg->state,msg->signal);
}
void QNode::StateMachineCallback3(const autopilots::StateMachine::ConstPtr &msg)
{
  Q_EMIT StateMachineSignal3(msg->state,msg->signal);
}
void QNode::QuadStateCallback1(const mavros_msgs::State::ConstPtr &msg)
{
  Q_EMIT QuadStateSignal1(msg->mode);
}
void QNode::QuadStateCallback2(const mavros_msgs::State::ConstPtr &msg)
{
  Q_EMIT QuadStateSignal2(msg->mode);
}
void QNode::QuadStateCallback3(const mavros_msgs::State::ConstPtr &msg)
{
  Q_EMIT QuadStateSignal3(msg->mode);
}

//void QNode::InterruptSlot1()
//{
//  ros::NodeHandle n;
//  n.setParam("/Quad1/state_machine/interruption", 1);
//}

//void QNode::ResumeSlot1()
//{
//  ros::NodeHandle n;
//  n.setParam("/Quad1/state_machine/resume", 1);
//}
//void QNode::InterruptSlot2()
//{
//  ros::NodeHandle n;
//  n.setParam("/Quad2/state_machine/interruption", 1);
//}

//void QNode::ResumeSlot2()
//{
//  ros::NodeHandle n;
//  n.setParam("/Quad2/state_machine/resume", 1);
//}
//void QNode::InterruptSlot3()
//{
//  ros::NodeHandle n;
//  n.setParam("/Quad3/state_machine/interruption", 1);
//}

//void QNode::ResumeSlot3()
//{
//  ros::NodeHandle n;
//  n.setParam("/Quad3/state_machine/resume", 1);
//}
void QNode::run() {
  ros::spin();
/*	ros::Rate loop_rate(1);
	int count = 0;
	while ( ros::ok() ) {

		std_msgs::String msg;
		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();
		chatter_publisher.publish(msg);
		log(Info,std::string("I sent: ")+msg.data);
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
  */
}

void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
        ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

}  // namespace qtros
