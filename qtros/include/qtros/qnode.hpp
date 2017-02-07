/**
 * @file /include/qtros/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef qtros_QNODE_HPP_
#define qtros_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/Altitude.h>
#include <sensor_msgs/BatteryState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <std_msgs/String.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include <ros/network.h>
#include <sstream>
#define PI 3.141592653


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qtros
{

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread
{
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();
  void chatterCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void chatterCallback2(const geometry_msgs::Twist::ConstPtr& msg);
  void AltitudeCallback1(const mavros_msgs::Altitude::ConstPtr& msg);
  void BatteryCallback1(const sensor_msgs::BatteryState::ConstPtr& msg);
  void PositionCallback1(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void VelocityCallback1(const geometry_msgs::TwistStamped::ConstPtr& msg);

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

Q_SIGNALS:
	void loggingUpdated();
  void rosShutdown();
  void CallBackTrigger(float,float);
  void CallBackTrigger2(float,float);
  void AltitudeSignal1(float);
  void BatterySignal1(float,float);
  void PositionSignal1(float,float,float);
  void VelocitySignal1(float,float,float);

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
  ros::Subscriber chatter_subscriber;
  ros::Subscriber chatter_subscriber2;
  ros::Subscriber AltitudeSubscriber1;
  ros::Subscriber BatterySubscriber1;
  ros::Subscriber PositionSubscriber1;
  ros::Subscriber VelocitySubscriber1;
  QStringListModel logging_model;

};
}
#endif /* qtros_QNODE_HPP_ */
