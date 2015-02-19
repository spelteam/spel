#include "frametablewidget.h"
#include <QHeaderView>
#include <QScrollBar>
#include <QLabel>
#include <QBitmap>
#include "project.h"
#include "utility.h"

FrameTableWidget::FrameTableWidget(QWidget *parent) :
    QTableWidget(parent)
{
    //have only one row
    this->setRowCount(1);
    this->setColumnCount(0);
    //hide row header
    this->verticalHeader()->hide();
    //fix column width
    this->horizontalHeader()->setSectionResizeMode(QHeaderView::Fixed);
}

#include <QDebug>
void FrameTableWidget::loadProjectEvent(){
    //load images
    auto paths = Project::getInstance().getPaths();
    auto projectFolder = Project::getInstance().getProjectFolder();
    for( int i = 0; i != paths.size(); i++ ){
        QImage img, mask;
        QString currImgPath = projectFolder+FilenamePath::imgFolderPath+paths[i].imgPath;
        QString currMaskPath = projectFolder+FilenamePath::maskFolderPath+paths[i].maskPath;
        if( img.load(currImgPath) && mask.load(currMaskPath) ){
            QLabel* item = new QLabel();
            QPixmap pixmap = QPixmap::fromImage(img)
                    .scaled(SCALE_FACTOR,SCALE_FACTOR);
            QBitmap bitmap = QBitmap::fromImage(mask)
                    .scaled(SCALE_FACTOR,SCALE_FACTOR)
                    .createMaskFromColor({0,0,0,255});
            pixmap.setMask(bitmap);
            item->setAlignment(Qt::AlignCenter);
            item->setPixmap(pixmap);

            //check whether keyframe
            if( Project::getInstance().getFrame(i)->getFrametype()== FRAMETYPE::KEYFRAME){
                item->setStyleSheet(Utility::fileToString(":/root/resources/stylesheets/Keyframe.qss"));
            } else{
                item->setStyleSheet(Utility::fileToString(":/root/resources/stylesheets/Interframe.qss"));
            }
            //add new column
            this->insertColumn(i);
            this->setCellWidget(0,i,item);
        } else{
            qDebug() << "FrameTableWidget: Error! Image not found!" << endl;
        }
    }

    qDebug() << "FrameTableWidget::loadProjectEvent" << endl;
    qDebug() << "width: " << this->horizontalHeader()->width()
             << "height: " << this->verticalHeader()->height()
             << endl;
    qDebug() << "widthCol: " << this->columnWidth(0) << endl;
    qDebug() << "heightCol: " << this->rowHeight(0) << endl;
    //set row height to table height
    this->resize(this->verticalHeader()->height(), this->width());
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
