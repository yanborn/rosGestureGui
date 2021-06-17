#ifndef GESTUREGUI_H
#define GESTUREGUI_H

#include "guihandler.h"

#include <QObject>
#include <QString>
#include <QtGui/QMainWindow>

namespace Ui {
class gesturegui;
}

class gesturegui : public QMainWindow
{
  Q_OBJECT

public:
  explicit gesturegui(int argc, char** argv, QWidget *parent = nullptr);
  ~gesturegui();

public Q_SLOTS:
  void on_connect_button_clicked();
  void leftDropdownHighlighted();
  void leftDropdownClicked();
  void leftDropdownDown();
  void leftDropdownUp();
  void rightDropdownHighlighted();
  void rightDropdownClicked();
  void rightDropdownDown();
  void rightDropdownUp();
  void sliderHighlighted();
  void sliderClicked();
  void sliderUp();
  void sliderDown();

private:
  Ui::gesturegui *ui;
  guiHandler guiHand;

  static bool leftDropdownIsHighlighted;
  static bool leftDropdownIsClicked;
  static bool rightDropdownIsHighlighted;
  static bool rightDropdownIsClicked;
  static bool sliderIsHighlighted;
  static bool sliderIsClicked;
  static QString const notHighlighted;
  static QString const highlighted;
};

#endif // GESTUREGUI_H
