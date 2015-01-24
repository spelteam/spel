#include "frametablewidget.h"
#include <QHeaderView>
#include <QScrollBar>

FrameTableWidget::FrameTableWidget(QWidget *parent) :
    QTableWidget(parent)
{
    //have only one row
    this->setRowCount(1);
    this->setColumnCount(5);
    //hide row header
    this->verticalHeader()->hide();
    //fix column width
    this->horizontalHeader()->setSectionResizeMode(QHeaderView::Fixed);
    //set style
    this->setStyleSheet("QTableWidget::item{ border : 5px solid blue }\nQTableWidget::item:focus{ border-color : red }");
}

#include <QDebug>
void FrameTableWidget::loadProjectEvent(){
    qDebug() << "FrameTableWidget::loadProjectEvent" << endl;
    return;
}

void FrameTableWidget::closeProjectEvent(){
    this->clear();
    this->setColumnCount(0);
}

void FrameTableWidget::createProjectEvent(){

}

void FrameTableWidget::resizeEvent(QResizeEvent *event){
    QTableWidget::resizeEvent(event);
    //set row height to table height
    QHeaderView* vertHeader = this->verticalHeader();
    vertHeader->setDefaultSectionSize(vertHeader->height());
}
