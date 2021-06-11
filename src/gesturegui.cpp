#include "gesturegui.h"
#include "ui_gesturegui.h"

#include <QObject>
#include <QComboBox>
#include <QPalette>
#include <QSlider>

bool gesturegui::leftDropdownIsHighlighted{false};
bool gesturegui::leftDropdownIsClicked{false};
bool gesturegui::rightDropdownIsHighlighted{false};
bool gesturegui::rightDropdownIsClicked{false};
bool gesturegui::sliderIsHighlighted{false};
bool gesturegui::sliderIsClicked{false};

QString const gesturegui::notHighlighted{"background-color:white"};
QString const gesturegui::highlighted{"background-color:grey"};

gesturegui::gesturegui(int argc, char** argv, QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::gesturegui),
  guiHand(argc, argv)
{
  ui->setupUi(this);

  //Set initial style for GUI Widgets
  ui->right_dropdown->setStyleSheet(notHighlighted);
  ui->left_dropdown->setStyleSheet(notHighlighted);
  ui->slider_frame->setStyleSheet(notHighlighted);

  QObject::connect(&guiHand, SIGNAL(rosShutdown()), this, SLOT(close()));
  QObject::connect(&guiHand, SIGNAL(leftDropdownHighlighted()), this, SLOT(leftDropdownHighlighted()));
  QObject::connect(&guiHand, SIGNAL(leftDropdownClicked()), this, SLOT(leftDropdownClicked()));
  QObject::connect(&guiHand, SIGNAL(rightDropdownHighlighted()), this, SLOT(rightDropdownHighlighted()));
  QObject::connect(&guiHand, SIGNAL(rightDropdownClicked()), this, SLOT(rightDropdownClicked()));
  QObject::connect(&guiHand, SIGNAL(sliderHighlighted()), this, SLOT(sliderHighlighted()));
  QObject::connect(&guiHand, SIGNAL(sliderClicked()), this, SLOT(sliderClicked()));
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

  if(!leftDropdownIsHighlighted && !rightDropdownIsClicked && !sliderIsClicked) {
    leftDropdownIsHighlighted = true;
    rightDropdownIsHighlighted = false;
    sliderIsHighlighted = false;
    ui->left_dropdown->setStyleSheet(highlighted);
    ui->right_dropdown->setStyleSheet(notHighlighted);
    ui->slider_frame->setStyleSheet(notHighlighted);
  }
  else {
    ROS_WARN_STREAM("Left dropdown already highlighted or other widget is still clicked");
  }
}

void
gesturegui::leftDropdownClicked()
{
  ROS_INFO_STREAM("Clicking on left dropdown menu");

  if(leftDropdownIsHighlighted && !rightDropdownIsClicked && !sliderIsClicked) {
    ui->left_dropdown->showPopup();
    ui->right_dropdown->hidePopup();
    leftDropdownIsClicked=!leftDropdownIsClicked;
  }
  else {
    ROS_WARN_STREAM("Left dropdown is not highlighted. Highlight before clicking on the left drop down.");
  }
}

void
gesturegui::rightDropdownHighlighted()
{
  ROS_INFO_STREAM("Highlighting right dropdown menu");

  if(!rightDropdownIsHighlighted && !leftDropdownIsClicked && !sliderIsClicked) {
    leftDropdownIsHighlighted = false;
    rightDropdownIsHighlighted = true;
    sliderIsHighlighted = false;
    ui->right_dropdown->setStyleSheet(highlighted);
    ui->left_dropdown->setStyleSheet(notHighlighted);
    ui->slider_frame->setStyleSheet(notHighlighted);
  }
  else {
    ROS_WARN_STREAM("Right dropdown already highlighted or other widget is still clicked");
  }

}

void
gesturegui::rightDropdownClicked()
{
  ROS_INFO_STREAM("Clicking on right dropdown menu");
  if(rightDropdownIsHighlighted) {
    ui->right_dropdown->showPopup();
    ui->left_dropdown->hidePopup();
    rightDropdownIsClicked=!rightDropdownIsClicked;
  }
  else {
    ROS_WARN_STREAM("Right dropdown is not highlighted. Highlight before clicking on the right drop down.");
  }
}

void
gesturegui::sliderHighlighted()
{
  ROS_INFO_STREAM("Highlighting slider");

  if(!sliderIsHighlighted && !leftDropdownIsClicked && !rightDropdownIsClicked) {
    leftDropdownIsHighlighted = false;
    rightDropdownIsHighlighted = false;
    sliderIsHighlighted = true;
    ui->right_dropdown->setStyleSheet(notHighlighted);
    ui->left_dropdown->setStyleSheet(notHighlighted);
    ui->slider_frame->setStyleSheet(highlighted);
  }
  else {
    ROS_WARN_STREAM("Slider is already highlighted or other Widget is still clicked");
  }
}

void
gesturegui::sliderClicked()
{
  ROS_INFO_STREAM("Clicking on slider");
  if(sliderIsHighlighted)
  {
    sliderIsClicked=!sliderIsClicked;
  }
  else {
    ROS_WARN_STREAM("Slider is not highlighted. Highlight before clicking on the slider.");
  }
}

void
gesturegui::sliderUp()
{
  ROS_INFO_STREAM("Setting slider one tick up");
  if(sliderIsClicked) {
    if(ui->slider->value() < ui->slider->maximum()) {
      ui->slider->setValue(ui->slider->value()+1);
    }
    else {
      ROS_WARN_STREAM("Slider already has maximum value");
    }
  }
  else {
    ROS_WARN_STREAM("Slider is not clicked. Click on slider before setting it one tick up");
  }
}

void
gesturegui::sliderDown()
{
  ROS_INFO_STREAM("Setting slider one tick down");
  if(sliderIsClicked) {
    if(ui->slider->value() > ui->slider->minimum()) {
      ui->slider->setValue(ui->slider->value()-1);
    }
    else {
      ROS_WARN_STREAM("Slider already has minimum value");
    }
  }
  else {
    ROS_WARN_STREAM("Slider is not clicked. Click on slider before setting it one tick down");
  }
}

