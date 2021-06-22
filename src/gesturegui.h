/**
 * @file gesturegui.h
 * @brief Header file for gesturegui class
 *
 * @author Yannick Volkenborn
 */

#ifndef GESTUREGUI_H
#define GESTUREGUI_H

#include "guihandler.h"

#include <QObject>
#include <QString>
#include <QtGui/QMainWindow>

namespace Ui {
/**
 * @brief The gesturegui class
 * @details Class for the QT based gesture gui
 */
class gesturegui;
}

class gesturegui : public QMainWindow
{
  Q_OBJECT

public:
  /**
   * @brief Constructor for gesturegui class
   *
   * @param argc Argument counter for initialization of guiHandler member class
   * @param argv Argument variables for initialization of guiHandler member class
   * @param parent QWidget object for initilization of QT GUI
   */
  explicit gesturegui(int argc, char** argv, QWidget *parent = nullptr);

  /**
   * @brief Destructor for gesturegui class
   */
  ~gesturegui();

public Q_SLOTS:
  /**
   * @brief Slot which will should be executed when clicking on connect button
   */
  void on_connect_button_clicked();

  /**
   * @brief Slot which will highlight the left dropdown menu
   * @details By highlighting the drop down menu it will be shown with a grey background
   */
  void leftDropdownHighlighted();

  /**
   * @brief Slot which will click on the left dropdown menu
   * @details The drop down menu can only be clicked when previously highlighted. By clicking on
   * the drop down menu the selections are displayed and the currently selected is ticked
   */
  void leftDropdownClicked();

  /**
   * @brief Slot which will move one index down on the left dropdown menu
   * @details Before moving the index the dropdown menu must be clicked. The index can only be moved
   * one tick at each operation. After moving one index the selection will be closed and must be clicked again
   */
  void leftDropdownDown();

  /**
   * @brief Slot which will move one index up on the left dropdown menu
   * @details Before moving the index the dropdown menu must be clicked. The index can only be moved
   * one tick at each operation. After moving one index the selection will be closed and must be clicked again
   */
  void leftDropdownUp();

  /**
   * @brief Slot which will highlight the right dropdown menu
   * @details By highlighting the drop down menu it will be shown with a grey background
   */
  void rightDropdownHighlighted();

  /**
   * @brief Slot which will click on the right dropdown menu
   * @details The drop down menu can only be clicked when previously highlighted. By clicking on
   * the drop down menu the selections are displayed and the currently selected is ticked
   */
  void rightDropdownClicked();

  /**
   * @brief Slot which will move one index down on the right dropdown menu
   * @details Before moving the index the dropdown menu must be clicked. The index can only be moved
   * one tick at each operation. After moving one index the selection will be closed and must be clicked again
   */
  void rightDropdownDown();

  /**
   * @brief Slot which will move one index up on the right dropdown menu
   * @details Before moving the index the dropdown menu must be clicked. The index can only be moved
   * one tick at each operation. After moving one index the selection will be closed and must be clicked again
   */
  void rightDropdownUp();

  /**
   * @brief Slot which will highlight the slider
   * @details By highlighting the slider it will be shown with a grey background
   */
  void sliderHighlighted();

  /**
   * @brief Slot which will click on the slider
   * @details The drop down menu can only be clicked when previously highlighted. By clicking on
   * the drop down menu you can move the slider up and down with the sliderUp() and sliderDown() methods
   */
  void sliderClicked();

  /**
   * @brief Slot which will move the slider one tick up
   * @details Before moving the slider must be clicked. The slider can be moved
   * one tick at each operation. After every operation the slider keeps clicked and must be unclicked
   * again.
   */
  void sliderUp();

  /**
   * @brief Slot which will move the slider one tick down
   * @details Before moving the slider must be clicked. The slider can be moved
   * one tick at each operation. After every operation the slider keeps clicked and must be unclicked
   * again.
   */
  void sliderDown();

private:
  /**
   * @brief gesturegui user interface object
   */
  Ui::gesturegui *ui;

  /**
   * @brief guiHandler member object for the listener thread
   */
  guiHandler guiHand;

  /**
   * @brief Boolean flag to indicate if the left dropdown menu is highlighted
   */
  static bool leftDropdownIsHighlighted;

  /**
   * @brief Boolean flag to indicate if the left dropdown menu is clicked
   */
  static bool leftDropdownIsClicked;

  /**
   * @brief Boolean flag to indicate if the right dropdown menu is highlighted
   */
  static bool rightDropdownIsHighlighted;

  /**
   * @brief Boolean flag to indicate if the right dropdown menu is clicked
   */
  static bool rightDropdownIsClicked;

  /**
   * @brief Boolean flag to indicate if the slider is highlighted
   */
  static bool sliderIsHighlighted;

  /**
   * @brief Boolean flag to indicate if the slider is clicked
   */
  static bool sliderIsClicked;

  /**
   * @brief QString for updating the widget stylesheet to indicate it is not highlighted
   */
  static QString const notHighlighted;

  /**
   * @brief QString for updating the widget stylesheet to indicate it is not highlighted
   */
  static QString const highlighted;
};

#endif // GESTUREGUI_H
