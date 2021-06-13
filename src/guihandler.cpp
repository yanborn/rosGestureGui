#include "gesturegui.h"
#include "guihandler.h"
#include "ros/console.h"
#include "ros/transport_hints.h"

#include <ros/ros.h>
#include <QApplication>

const std::string guiHandler::leftDropdownHighlightedMsg{"leftHighlighted"};
const std::string guiHandler::leftDropdownClickedMsg{"leftClicked"};
const std::string guiHandler::rightDropdownHighlightedMsg{"rightHighlighted"};
const std::string guiHandler::rightDropdownClickedMsg{"rightClicked"};
const std::string guiHandler::sliderHighlightedMsg{"sliderHighlighted"};
const std::string guiHandler::sliderClickedMsg{"sliderClicked"};
const std::string guiHandler::sliderUpMsg{"sliderUp"};
const std::string guiHandler::sliderDownMsg{"sliderDown"};
const std::string guiHandler::closeGuiMsg{"closeGui"};

const std::string guiHandler::nodeName{"gestureGui"};

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
  ROS_INFO_STREAM("Start initializing guiHandler");
  ros::init(init_argc,init_argv,"gestureGui");

  std::size_t counter{0};

  while(!ros::master::check()) {
    counter++;
    ros::Duration(0.5).sleep();
    if(counter > 6) {
      return false;
    }
  }

  ros::start();

  ROS_INFO_STREAM("Setting up nodehandler");
  ros::NodeHandle n;
  gesture_subscriber = n.subscribe(nodeName, nodeQueueSize, &guiHandler::handleCallback, this);

  ROS_INFO_STREAM("Starting thread for listener on " << nodeName << " topic");
  start();
  return true;
}

void
guiHandler::run()
{
  ROS_INFO_STREAM("Thread for listener on " << nodeName << " topic started");
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
  ROS_INFO_STREAM("Message received: " << msg->data.c_str());

  if(msg->data.c_str() == leftDropdownClickedMsg) {
    Q_EMIT leftDropdownClicked();
  }
  else if(msg->data.c_str() == rightDropdownClickedMsg) {
    Q_EMIT rightDropdownClicked();
  }
  else if(msg->data.c_str() == closeGuiMsg) {
    ROS_INFO_STREAM("Closing gesture GUI");
    Q_EMIT closeGui();
  }
  else if(msg->data.c_str() == sliderUpMsg) {
    Q_EMIT sliderUp();
  }
  else if(msg->data.c_str() == sliderDownMsg) {
    Q_EMIT sliderDown();
  }
  else if(msg->data.c_str() == leftDropdownHighlightedMsg) {
    Q_EMIT leftDropdownHighlighted();
  }
  else if(msg->data.c_str() == rightDropdownHighlightedMsg) {
    Q_EMIT rightDropdownHighlighted();
  }
  else if(msg->data.c_str() == sliderHighlightedMsg) {
    Q_EMIT sliderHighlighted();
  }
  else if(msg->data.c_str() == sliderClickedMsg) {
    Q_EMIT sliderClicked();
  }
  else {
    ROS_ERROR_STREAM("Message not handled by guiHandler");
  }
}
