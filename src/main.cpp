/**
 * @file main.cpp
 * @brief Main file to run the gesture gui application
 *
 * @author Yannick Volkenborn
 */

#include "gesturegui.h"

#include <QApplication>
#include <ros/ros.h>

int
main(int argc, char **argv)
{
  ROS_INFO("ROS Gesture GUI started!");

  QApplication app(argc, argv);
  gesturegui gui(argc, argv);
  gui.show();
  return app.exec();
}
