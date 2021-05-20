#include "guihandler.h"
#include "ros/console.h"


guiHandler::guiHandler()
{
}

void guiHandler::handleCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO_STREAM("Message reveiced: " << *msg);
}
