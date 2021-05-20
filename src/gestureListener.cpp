#include "gesturegui.h"

#include <ros/ros.h>
#include <QApplication>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "gestureListener");
  ros::NodeHandle nh;

  ROS_INFO("Hello world!");

  QApplication app(argc, argv);

  gesturegui gui;

  gui.show();
  return app.exec();
}
