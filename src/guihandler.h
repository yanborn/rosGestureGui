#ifndef GUIHANDLER_H
#define GUIHANDLER_H

#include "gesturegui.h"
#include "std_msgs/String.h"

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <QThread>

class guiHandler : public QThread
{
  Q_OBJECT
public:
  //Constructor
  guiHandler(int argc, char** argv);

  //Constructor
  virtual ~guiHandler();

  bool init();
  void run();

  //Callback to register with the subscriber node
  static void handleCallback(const std_msgs::String::ConstPtr& msg);

Q_SIGNALS:
  void rosShutdown();

private:
  int init_argc;
  char** init_argv;
  ros::Subscriber gesture_subscriber;
  static const std::string leftDropdownClicked;
};

#endif // GUIHANDLER_H
