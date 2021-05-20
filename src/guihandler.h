#ifndef GUIHANDLER_H
#define GUIHANDLER_H

#include "std_msgs/String.h"

class guiHandler
{
public:
  //Constructor
  guiHandler();

  //Constructor
  ~guiHandler() = default;

  //Callback to register with the subscriber node
  static void handleCallback(const std_msgs::String::ConstPtr& msg);

private:

};

#endif // GUIHANDLER_H
