/**
 * @file guihandler.h
 * @brief Header file for guihandler class
 *
 * @author Yannick Volkenborn
 */

#ifndef GUIHANDLER_H
#define GUIHANDLER_H

#include "cstring"
#include "gesturegui.h"
#include "std_msgs/String.h"

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <QThread>

/**
 * @brief The guiHandler class
 * @details Class to handle the incoming gui messages via the gestureGui topic
 */
class guiHandler : public QThread
{
  Q_OBJECT
public:
  /**
   * @brief Constructor for guiHandler class
   *
   * @param argc Argument counter for initialization of the ros node
   * @param argv Argument variables for initialization of the ros node
   */
  guiHandler(int argc, char** argv);

  /**
   * @brief Destructor for guiHandler class
   */
  virtual ~guiHandler();

  /**
   * @brief Initializes the ros node and starts the topic listener thread
   * @details Initializes the ros node, sets the nodehandle, registers the callback and starts the QThread
   * which uses the run method.
   *
   * @return true if thread was successfully started
   * @return false if ros::master::check was false 6 times in a row
   */
  bool init();

  /**
   * @brief Method which will be executed when starting the thread
   * @details Runs until ros::ok() will be false
   */
  void run();

  //Callback to register with the subscriber node
  /**
   * @brief Callback to handle the incoming topic messages
   * @details Emits the signals depending on the received topic messages
   *
   * @param msg Topic message reference
   */
  void handleCallback(const std_msgs::String::ConstPtr& msg);

Q_SIGNALS:
  /**
   * @brief Signal to indicate a ros shutdown
   */
  void rosShutdown();

  /**
   * @brief Signal to indicate a highlighting for the left drop down menu
   */
  void leftDropdownHighlighted();

  /**
   * @brief Signal to indicate a click on the left drop down menu
   */
  void leftDropdownClicked();

  /**
   * @brief Signal to indicate a down on the left drop down menu
   */
  void leftDropdownDown();

  /**
   * @brief Signal to indicate an up on the left drop down menu
   */
  void leftDropdownUp();

  /**
   * @brief Signal to indicate a highlighting for the right drop down menu
   */
  void rightDropdownHighlighted();

  /**
   * @brief Signal to indicate a click on the right drop down menu
   */
  void rightDropdownClicked();

  /**
   * @brief Signal to indicate a down on the right drop down menu
   */
  void rightDropdownDown();

  /**
   * @brief Signal to indicate an up on the right drop down menu
   */
  void rightDropdownUp();

  /**
   * @brief Signal to indicate a highlighting for the slider
   */
  void sliderHighlighted();

  /**
   * @brief Signal to indicate a click on the slider
   */
  void sliderClicked();

  /**
   * @brief Signal to indicate a down on the slider
   */
  void sliderDown();

  /**
   * @brief Signal to indicate an up on the slider
   */
  void sliderUp();

  /**
   * @brief Signal to indicate a closure of the GUI
   */
  void closeGui();

private:
  /**
   * @brief Topic subscriber object
   */
  ros::Subscriber gesture_subscriber;

  /**
   *@brief Size of the node queue
   */
  static constexpr std::uint32_t nodeQueueSize{1000};

  /**
   * @brief Initial argument counter
   */
  int init_argc;

  /**
   * @brief Initial argument variables
   */
  char** init_argv;

  /**
   * @brief Topic message for highlighting the left drop down menu
   */
  static const std::string leftDropdownHighlightedMsg;

  /**
   * @brief Topic message for clicking on the left drop down menu
   */
  static const std::string leftDropdownClickedMsg;

  /**
   * @brief Topic message for setting the left drop down menu one index down
   */
  static const std::string leftDropdownDownMsg;

  /**
   * @brief Topic message for setting the left drop down menu one index up
   */
  static const std::string leftDropdownUpMsg;

  /**
   * @brief Topic message for highlighting the right drop down menu
   */
  static const std::string rightDropdownHighlightedMsg;

  /**
   * @brief Topic message for clicking on the right drop down menu
   */
  static const std::string rightDropdownClickedMsg;

  /**
   * @brief Topic message for setting the right drop down menu one index down
   */
  static const std::string rightDropdownDownMsg;

  /**
   * @brief Topic message for setting the right drop down menu one index up
   */
  static const std::string rightDropdownUpMsg;

  /**
   * @brief Topic message for highlighting the slider
   */
  static const std::string sliderHighlightedMsg;

  /**
   * @brief Topic message for clicking on the slider
   */
  static const std::string sliderClickedMsg;

  /**
   * @brief Topic message for setting the slider one tick up
   */
  static const std::string sliderUpMsg;

  /**
   * @brief Topic message for setting the slider one tick down
   */
  static const std::string sliderDownMsg;

  /**
   * @brief Topic message for closing the GUI
   */
  static const std::string closeGuiMsg;

  /**
   * @brief Node name for the gui handler
   */
  static const std::string nodeName;
};

#endif // GUIHANDLER_H
