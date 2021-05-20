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

  ros::Subscriber sub = nh.subscribe("gestureGui", 1000, guiHandler::handleCallback);

  ros::spin();

  QApplication app(argc, argv);

  gesturegui gui;

  gui.show();
  return app.exec();

}
