#include "gesturegui.h"
#include "guihandler.h"
#include "ros/console.h"

#include <ros/ros.h>
#include <QApplication>

const std::string guiHandler::leftDropdownClicked{"left"};

guiHandler::guiHandler(int argc, char** argv):
  init_argc(argc),
  init_argv(argv)
{
}

guiHandler::~guiHandler()
{
  if(ros::isStarted()) {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  wait();
}

bool
guiHandler::init()
{
  ros::init(init_argc,init_argv,"qtgui");
  if ( ! ros::master::check() ) {
    return false;
  }
  ros::start();
  ros::NodeHandle n;
  gesture_subscriber = n.subscribe("gestureGui", 1000, handleCallback);
  start();
  return true;
}

void
guiHandler::run()
{
  ros::Rate loop_rate(10);

  int count = 0;
  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }

  Q_EMIT rosShutdown();
}

void guiHandler::handleCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO_STREAM("Message reveiced: " << msg->data.c_str());

  if(msg->data.c_str() == leftDropdownClicked) {
    ROS_INFO_STREAM("I handled the message");
  }
  else {
    ROS_ERROR_STREAM("Message not handled by guiHandler");
  }
}
