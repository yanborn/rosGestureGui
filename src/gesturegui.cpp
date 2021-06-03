#include "gesturegui.h"
#include "ui_gesturegui.h"

#include <QObject>

gesturegui::gesturegui(int argc, char** argv, QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::gesturegui),
  gui(argc, argv)
{
  ui->setupUi(this);

  QObject::connect(&gui, SIGNAL(rosShutdown()), this, SLOT(close()));
  QObject::connect(&gui, SIGNAL(leftDropdownClicked()), this, SLOT(on_left_dropdown_clicked()));
}

gesturegui::~gesturegui()
{
  delete ui;
}

void
gesturegui::on_connect_button_clicked()
{
  if(!gui.init()) {
    ROS_ERROR_STREAM("Error starting topic listener");
  }
  else {
    gui.run();
  }
}

void
gesturegui::on_left_dropdown_clicked()
{
  ROS_INFO_STREAM("Signaltest");
}
