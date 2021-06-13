#include "gesturegui.h"
#include "guihandler.h"

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
