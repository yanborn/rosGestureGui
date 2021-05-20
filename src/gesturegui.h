#ifndef GESTUREGUI_H
#define GESTUREGUI_H

#include <QMainWindow>

namespace Ui {
class gesturegui;
}

class gesturegui : public QMainWindow
{
  Q_OBJECT

public:
  explicit gesturegui(QWidget *parent = nullptr);
  ~gesturegui();

private:
  Ui::gesturegui *ui;
};

#endif // GESTUREGUI_H
