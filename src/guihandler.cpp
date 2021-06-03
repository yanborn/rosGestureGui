#include "gesturegui.h"
#include "guihandler.h"
#include "ros/console.h"
#include "ros/transport_hints.h"

#include <ros/ros.h>
#include <QApplication>

const std::string guiHandler::leftDropdownClickedMsg{"left"};
const std::string guiHandler::nodeName{"gestureGui"};
const std::uint32_t guiHandler::nodeQueueSize{10};

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
  ros::TransportHints th;
  gesture_subscriber = n.subscribe(nodeName, nodeQueueSize, &guiHandler::handleCallback, this);
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

void
guiHandler::handleCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO_STREAM("Message reveiced: " << msg->data.c_str());

  if(msg->data.c_str() == leftDropdownClickedMsg) {
    ROS_INFO_STREAM("I handled the message");
    Q_EMIT leftDropdownClicked();
  }
  else {
    ROS_ERROR_STREAM("Message not handled by guiHandler");
  }
}
