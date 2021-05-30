#include "gesturegui.h"
#include "guihandler.h"

#include <QApplication>
#include <ros/ros.h>

int
main(int argc, char **argv)
{
  ros::init(argc, argv, "gestureListener");
  ros::NodeHandle nh;

  ROS_INFO("ROS Gesture GUI started!");

  QApplication app(argc, argv);

  guiHandler handler;

  handler.gui.show();

  ros::Subscriber sub = nh.subscribe("gestureGui", 1000, handler.handleCallback);
  ros::spin();
  app.exec();

  return 1;

}
