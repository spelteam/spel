#include "frameview2d.h"
#include "bodyjointitem.h"
#include "project.h"
#include <QBitmap>

//PUBLIC
#include <QDebug>

FrameView2D::FrameView2D(QWidget *parent) :
    QWidget(parent)
{
    scene = new QGraphicsScene;
    view = new TestGraphicsView(scene,parent);
}

FrameView2D::~FrameView2D(){
    delete view;
    delete scene;
}

void FrameView2D::loadProjectEvent(){
    //EMPTY
}

void FrameView2D::closeProjectEvent(){
    scene->clear();
    view->update();
}

void FrameView2D::scaleItemsEvent(int value){
    //TEMP
    auto itemList = scene->items();
    for( auto it = itemList.begin(); it != itemList.end(); it++ ){
        if( dynamic_cast<BodyJointItem*>(*it) ){
            (*it)->setScale(value/10.f);
        }
    }
    scene->update();
}

void FrameView2D::changeMaskOpacityEvent(int value){
    //TEMP
}

void FrameView2D::pickFrameEvent(int, int col){
    auto paths = Project::getInstance().getPaths();
    auto projectFolder = Project::getInstance().getProjectFolder();
    QImage img, mask;
    QString imgPath = projectFolder+FilenamePath::imgFolderPath+paths[col].imgPath;
    QString maskPath = projectFolder+FilenamePath::maskFolderPath+paths[col].maskPath;
    auto itemList = scene->items();
    if( img.load(imgPath) && mask.load(maskPath) ){
        bool exist = false;
        for( auto it = itemList.begin(); it != itemList.end(); it++){
            if( dynamic_cast<QGraphicsPixmapItem*>(*it) ){
                auto item = static_cast<QGraphicsPixmapItem*>(*it);
                QPixmap pixmap = QPixmap::fromImage(img);
                QBitmap bitmap = QBitmap::fromImage(mask)
                        .createMaskFromColor({0,0,0,255});
                pixmap.setMask(bitmap);
                item->setPixmap(pixmap);
                exist = true;
                break;
            }
        }
        if(!exist){
            QPixmap pixmap = QPixmap::fromImage(img);
            QBitmap bitmap = QBitmap::fromImage(mask)
                    .createMaskFromColor({0,0,0,255});
            pixmap.setMask(bitmap);
            scene->addPixmap(pixmap);
        }
        view->update();
    } else{
        qDebug() << "FrameView2D: Error! Image not found!" << endl;
    }
    //clear all body joints
    for( auto it = itemList.begin(); it != itemList.end(); ++it){
        if( dynamic_cast<BodyJointItem*>(*it) ){
            scene->removeItem(*it);
        }
    }
    //load body joints
    auto frame = Project::getInstance().getFrame(col);
    if( frame->getFrametype() == KEYFRAME ){
        auto bodyJoints = frame->getSkeleton()
                .getJointTree();
        auto it = bodyJoints.begin();
        while( it != bodyJoints.end() ){
            BodyJoint joint = *it;
            BodyJointItem* newItem = new BodyJointItem();
            newItem->setPos(joint.getImageLocation().x,joint.getImageLocation().y);
            newItem->setZValue(10.0);
            scene->addItem(newItem);
            ++it;
        }
    }
}

//PRIVATE
#include <QWheelEvent>
void FrameView2D::TestGraphicsView::wheelEvent(QWheelEvent *event){
    setTransformationAnchor(QGraphicsView::AnchorUnderMouse);

    // Scale the view / do the zoom
    double scaleFactor = 1.15;
    if(event->delta() > 0) {
        // Zoom in
        scale(scaleFactor, scaleFactor);
    } else {
        // Zooming out
        scale(1.0 / scaleFactor, 1.0 / scaleFactor);
    }
}


