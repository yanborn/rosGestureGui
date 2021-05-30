#ifndef GUIHANDLER_H
#define GUIHANDLER_H

#include "gesturegui.h"
#include "std_msgs/String.h"

#include <QApplication>

class guiHandler
{
public:
  //Constructor
  guiHandler();

  //Constructor
  ~guiHandler() = default;

  //Callback to register with the subscriber node
  static void handleCallback(const std_msgs::String::ConstPtr& msg);

  gesturegui gui;
private:

  static const std::string leftDropdownClicked;
};

#endif // GUIHANDLER_H
