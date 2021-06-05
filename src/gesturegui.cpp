#include "gesturegui.h"
#include "ui_gesturegui.h"

#include <QObject>

gesturegui::gesturegui(int argc, char** argv, QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::gesturegui),
  guiHand(argc, argv)
{
  ui->setupUi(this);

  QObject::connect(&guiHand, SIGNAL(rosShutdown()), this, SLOT(close()));
  QObject::connect(&guiHand, SIGNAL(leftDropdownClicked()), this, SLOT(left_dropdown_click()));
}

gesturegui::~gesturegui()
{
  delete ui;
}

void
gesturegui::on_connect_button_clicked()
{
  if(!guiHand.init()) {
    ROS_ERROR_STREAM("Error starting topic listener");
  }
}

void
gesturegui::left_dropdown_click()
{
}
