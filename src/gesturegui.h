#ifndef GESTUREGUI_H
#define GESTUREGUI_H

#include "guihandler.h"

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

  //int select_left_combo();
public Q_SLOTS:
  void on_connect_button_clicked();
  void on_left_dropdown_clicked();

private:
  Ui::gesturegui *ui;
  guiHandler gui;
};

#endif // GESTUREGUI_H
