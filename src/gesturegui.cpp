#include "gesturegui.h"
#include "ui_gesturegui.h"

#include <QObject>
#include <QComboBox>
#include <QPalette>
#include <QSlider>

gesturegui::gesturegui(int argc, char** argv, QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::gesturegui),
  guiHand(argc, argv)
{
  ui->setupUi(this);

  //Set initial style for GUI Widgets
  ui->right_dropdown->setStyleSheet("background-color:white");
  ui->left_dropdown->setStyleSheet("background-color:white");
  ui->slider_frame->setStyleSheet("background-color:white");

  QObject::connect(&guiHand, SIGNAL(rosShutdown()), this, SLOT(close()));
  QObject::connect(&guiHand, SIGNAL(leftDropdownHighlighted()), this, SLOT(leftDropdownHighlighted()));
  QObject::connect(&guiHand, SIGNAL(leftDropdownClicked()), this, SLOT(leftDropdownClicked()));
  QObject::connect(&guiHand, SIGNAL(rightDropdownHighlighted()), this, SLOT(rightDropdownHighlighted()));
  QObject::connect(&guiHand, SIGNAL(rightDropdownClicked()), this, SLOT(rightDropdownClicked()));
  QObject::connect(&guiHand, SIGNAL(sliderUp()), this, SLOT(sliderUp()));
  QObject::connect(&guiHand, SIGNAL(sliderDown()), this, SLOT(sliderDown()));
  QObject::connect(&guiHand, SIGNAL(closeGui()), this, SLOT(close()));
}

gesturegui::~gesturegui()
{
  delete ui;
}

void
gesturegui::on_connect_button_clicked()
{
  if(!guiHand.init()) {
    ROS_ERROR_STREAM("Error starting ros node");
  }
}

void
gesturegui::leftDropdownHighlighted()
{
  ROS_INFO_STREAM("Highlighting left dropdown menu");
  ui->left_dropdown->setStyleSheet("background-color:grey");
  ui->right_dropdown->setStyleSheet("background-color:white");
  ui->slider_frame->setStyleSheet("background-color:white");
}

void
gesturegui::leftDropdownClicked()
{
  ROS_INFO_STREAM("Clicking on left dropdown menu");
  ui->left_dropdown->showPopup();
  ui->right_dropdown->hidePopup();
}

void
gesturegui::rightDropdownHighlighted()
{
  ROS_INFO_STREAM("Highlighting right dropdown menu");
  ui->right_dropdown->setStyleSheet("background-color:grey");
  ui->left_dropdown->setStyleSheet("background-color:white");
  ui->slider_frame->setStyleSheet("background-color:white");
}

void
gesturegui::rightDropdownClicked()
{
  ROS_INFO_STREAM("Clicking on right dropdown menu");
  ui->right_dropdown->showPopup();
  ui->left_dropdown->hidePopup();
}

void
gesturegui::sliderUp()
{
  ROS_INFO_STREAM("Setting slider one tick up");
  if(ui->slider->value() < ui->slider->maximum()) {
    ui->slider->setValue(ui->slider->value()+1);
  }
  else {
    ROS_ERROR_STREAM("Slider already has maximum value");
  }
}

void
gesturegui::sliderDown()
{
  ROS_INFO_STREAM("Setting slider one tick down");
  if(ui->slider->value() > ui->slider->minimum()) {
    ui->slider->setValue(ui->slider->value()-1);
  }
  else {
    ROS_ERROR_STREAM("Slider already has minimum value");
  }
}
