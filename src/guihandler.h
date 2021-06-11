#ifndef GUIHANDLER_H
#define GUIHANDLER_H

#include "cstring"
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
  void handleCallback(const std_msgs::String::ConstPtr& msg);

Q_SIGNALS:
  void rosShutdown();
  void leftDropdownHighlighted();
  void leftDropdownClicked();
  void rightDropdownHighlighted();
  void rightDropdownClicked();
  void sliderHighlighted();
  void sliderClicked();
  void sliderUp();
  void sliderDown();

  void closeGui();

private:
  //Topic listener object
  ros::Subscriber gesture_subscriber;

  //Initialization values
  static constexpr std::uint32_t nodeQueueSize{1000};
  int init_argc;
  char** init_argv;

  //Topic Messages
  static const std::string leftDropdownHighlightedMsg;
  static const std::string leftDropdownClickedMsg;
  static const std::string rightDropdownHighlightedMsg;
  static const std::string rightDropdownClickedMsg;
  static const std::string sliderHighlightedMsg;
  static const std::string sliderClickedMsg;
  static const std::string sliderUpMsg;
  static const std::string sliderDownMsg;
  static const std::string closeGuiMsg;

  static const std::string nodeName;
};

#endif // GUIHANDLER_H
