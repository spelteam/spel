// SPEL definitions
#include "predef.hpp"

#include "frametablewidget.h"
#include <QHeaderView>
#include <QScrollBar>
#include <QLabel>
#include <QBitmap>
#include <QKeyEvent>
#include "project.h"
#include "utility.h"

using posegui::Project;

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
  //emit pickFrame signal
  QObject::connect(this, &FrameTableWidget::cellClicked, this, &FrameTableWidget::pickFrameEventEmitter);
  QObject::connect(this, &FrameTableWidget::cellActivated, this, &FrameTableWidget::pickFrameEventEmitter);
  QObject::connect(this, &FrameTableWidget::cellEntered, this, &FrameTableWidget::pickFrameEventEmitter);
}

#include <QDebug>
void FrameTableWidget::loadProjectEvent(){
  qDebug() << "Frame table view loading" << endl;
  //load images
  auto projectPaths = Project::getInstance().getPaths();
  auto projectFolder = Project::getInstance().getProjectFolder();
  for (int i = 0; i != projectPaths.paths.size(); i++){
    QImage img, mask;
    QString currImgPath = projectFolder + projectPaths.imgFolderPath + projectPaths.paths[i].imgPath;
    QString currMaskPath = projectFolder + projectPaths.maskFolderPath + projectPaths.paths[i].maskPath;
    if (img.load(currImgPath) && mask.load(currMaskPath)){
      QLabel* item = new QLabel();
      QPixmap pixmap = QPixmap::fromImage(img)
        .scaled(SCALE_FACTOR, SCALE_FACTOR);
      QBitmap bitmap = QBitmap::fromImage(mask)
        .scaled(SCALE_FACTOR, SCALE_FACTOR)
        .createMaskFromColor({ 0, 0, 0, 255 });
      pixmap.setMask(bitmap);
      item->setAlignment(Qt::AlignCenter);
      item->setPixmap(pixmap);

      //check whether keyframe
      if (Project::getInstance().getFrame(i)->getFrametype() == FRAMETYPE::KEYFRAME){
        item->setStyleSheet(posegui::Utility::fileToString(":/root/resources/stylesheets/Keyframe.qss"));
      }
      else{
        item->setStyleSheet(posegui::Utility::fileToString(":/root/resources/stylesheets/Interframe.qss"));
      }
      //add new column
      this->insertColumn(i);
      this->setCellWidget(0, i, item);
    }
    else{
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

void FrameTableWidget::changeFrametypeEvent(int num){
  //change style of cell
  auto frame = Project::getInstance().getFrame(num);
  if (frame->getFrametype() == KEYFRAME){
    this->cellWidget(0, num)
      ->setStyleSheet(posegui::Utility::fileToString(":/root/resources/stylesheets/Keyframe.qss"));
  }
  else{
    this->cellWidget(0, num)
      ->setStyleSheet(posegui::Utility::fileToString(":/root/resources/stylesheets/Interframe.qss"));
  }
  //repick frame
  pickFrame(num);
}

//TODO: [L] Fix image resizing
void FrameTableWidget::resizeEvent(QResizeEvent *event){
  QTableWidget::resizeEvent(event);
  //set row height to table height
  QHeaderView* vertHeader = this->verticalHeader();
  vertHeader->setDefaultSectionSize(vertHeader->height());
}

void FrameTableWidget::keyPressEvent(QKeyEvent *event){
  if (Project::getInstance().getState() == Project::ProjectState::LOADED){
    switch (event->key()) {
    case Qt::Key_Left:
    case Qt::Key_B:{
      QModelIndexList selectedCells = this->selectedIndexes();
      //init with last column
      int selectedColumn = this->columnCount() - 1;
      int columnCount = this->columnCount();
      if (!selectedCells.empty()){
        //get first selected cell
        selectedColumn = selectedCells
          .at(0)
          .column();
      }
      //go to prev
      selectedColumn = (selectedColumn - 1 + columnCount) % columnCount;
      //select cell
      this->selectColumn(selectedColumn);
      //click on the selected cell
      this->cellClicked(0, selectedColumn);
      break;
    }
    case Qt::Key_Right:
    case Qt::Key_N:{
      QModelIndexList selectedCells = this->selectedIndexes();
      //init with first column
      int selectedColumn = 0;
      int columnCount = this->columnCount();
      if (!selectedCells.empty()){
        //get last selected cell
        selectedColumn = selectedCells
          .at(selectedCells.size() - 1)
          .column();
      }
      //go to next
      selectedColumn = (selectedColumn + 1) % columnCount;
      //select cell
      this->selectColumn(selectedColumn);
      //click on the selected cell
      this->cellClicked(0, selectedColumn);
      break;
    }
    default:{
      break;
    }
    }
  }
}

//PRIVATE

void FrameTableWidget::pickFrameEventEmitter(int, int col){
  emit pickFrame(col);
}
