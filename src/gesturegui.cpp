#include "gesturegui.h"
#include "ui_gesturegui.h"

gesturegui::gesturegui(int argc, char** argv, QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::gesturegui),
  gui(argc, argv)
{
  ui->setupUi(this);

  QObject::connect(&gui, SIGNAL(rosShutdown()), this, SLOT(close()));
}

gesturegui::~gesturegui()
{
  delete ui;
}

void gesturegui::on_connect_button_clicked()
{
  if(!gui.init()) {
    ROS_ERROR_STREAM("Error starting topic listener");
  }
  else {
    gui.run();
  }
}
