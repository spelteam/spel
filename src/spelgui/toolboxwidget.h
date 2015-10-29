#ifndef TOOLBOXWIDGET_H
#define TOOLBOXWIDGET_H

// SPEL definitions
#include "predef.hpp"

#include <QGroupBox>
#include "flowlayout.h"
#include <QPushButton>

class ToolBoxWidget : public QGroupBox
{
  Q_OBJECT
public:
  explicit ToolBoxWidget(QWidget *parent = 0);
  virtual ~ToolBoxWidget();
signals:

  public slots :
private:
  //layouts
  FlowLayout *MainLayout;
  //view
  QPushButton *eraser;
  QPushButton *picker;
  QPushButton *jointer;
  QPushButton *bodyParter;

};

#endif // TOOLBOXWIDGET_H
