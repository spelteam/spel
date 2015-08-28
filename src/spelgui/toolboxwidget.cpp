#include "toolboxwidget.h"

ToolBoxWidget::ToolBoxWidget(QWidget *parent) :
QGroupBox(parent)
{
  //view
  eraser = new QPushButton(this);
  eraser->setIcon(QPixmap(QString(":/root/resources/icons/eraser.png")));
  eraser->setToolTip("Eraser");

  picker = new QPushButton(this);
  picker->setIcon(QPixmap(QString(":/root/resources/icons/picker.png")));
  picker->setToolTip("Select&Move");

  jointer = new QPushButton(this);
  jointer->setIcon(QPixmap(QString(":/root/resources/icons/jointer.png")));
  jointer->setToolTip("Create body joint");

  bodyParter = new QPushButton(this);
  bodyParter->setIcon(QPixmap(QString(":/root/resources/icons/body_parter.png")));
  bodyParter->setToolTip("Create body part");

  //layouts
  MainLayout = new FlowLayout;
  MainLayout->addWidget(picker);
  MainLayout->addWidget(eraser);
  MainLayout->addWidget(jointer);
  MainLayout->addWidget(bodyParter);
  //settings
  this->setEnabled(false);
  this->setLayout(MainLayout);
  //connect
  //QObject::connect(eraser,&QLabel::mousePressEvent,this,&ToolBoxWidget::mousePressEvent);

}

ToolBoxWidget::~ToolBoxWidget(){
  delete eraser;
  delete picker;
  delete MainLayout;
}
