#include "gesturegui.h"
#include "ui_gesturegui.h"

gesturegui::gesturegui(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::gesturegui)
{
  ui->setupUi(this);
}

gesturegui::~gesturegui()
{
  delete ui;
}
