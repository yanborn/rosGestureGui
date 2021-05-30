#include "gesturegui.h"
#include "guihandler.h"
#include "ros/console.h"

#include <QApplication>


guiHandler::guiHandler()
{
}

const std::string guiHandler::leftDropdownClicked{"left"};

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
