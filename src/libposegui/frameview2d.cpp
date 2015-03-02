#include "frameview2d.h"
#include "bodyjointitem.h"
#include "bodypartitem.h"
#include "project.h"
#include "utility.h"
#include <QBitmap>
#include <QWheelEvent>

//PUBLIC
#include <QDebug>

FrameView2D::FrameView2D(QWidget *parent) :
    QWidget(parent),
    frameImage(nullptr)
{
    scene = new QGraphicsScene;
    scene->setItemIndexMethod(QGraphicsScene::NoIndex);
    view = new FrameGraphicsView(scene,parent);
}

FrameView2D::~FrameView2D(){
    delete view;
    delete scene;
}

void FrameView2D::loadProjectEvent(){
    qDebug() << "Frame view loading" << endl;
    //EMPTY
}

#include <QDebug>
void FrameView2D::closeProjectEvent(){
    scene->clear();
    view->update();
    //no image exist on view
    frameImage = nullptr;
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
    loadFrameImage(col);
    loadFrameJoints(col);
}

//PRIVATE

void FrameView2D::loadFrameImage(int num){
    FilenamePath paths = Project::getInstance().getPaths()[num];
    auto projectFolder = Project::getInstance().getProjectFolder();
    QString imgPath = projectFolder+FilenamePath::imgFolderPath+paths.imgPath;
    QString maskPath = projectFolder+FilenamePath::maskFolderPath+paths.maskPath;

    QImage img, mask;
    if( img.load(imgPath) && mask.load(maskPath) ){
        //TODO: [L] Write pixmap here!
        if(frameImage){
            //frameImage exist on view
            QPixmap pixmap = QPixmap::fromImage(img);
            /*QBitmap bitmap = QBitmap::fromImage(mask)
                    .createMaskFromColor({0,0,0,255});
            pixmap.setMask(bitmap);*/
            frameImage->setPixmap(pixmap);
        } else{
            //frameImage not exist on view
            QPixmap pixmap = QPixmap::fromImage(img);
           /* QBitmap bitmap = QBitmap::fromImage(mask)
                    .createMaskFromColor({0,0,0,255});
            pixmap.setMask(bitmap);*/
            frameImage = scene->addPixmap(pixmap);
            frameImage->setZValue(-1.0);
            //set scene bounded rect
            scene->setSceneRect(pixmap.rect());
        }
        view->update();
    } else{
        qDebug() << "FrameView2D: Error! Image not found!" << endl;
    }
}

//TODO: [!] Load skeleton, (save?)
void FrameView2D::loadFrameJoints(int num){
    //clear frame
    auto itemList = scene->items();
    if( itemList.size() > 1 ){
        //more then one background image
        for( auto it = itemList.begin(); it != itemList.end(); ++it){
            if( Utility::isSkeletonItem(it) ){
                scene->removeItem(*it);
            }
        }
    }
    //load skeleton
    auto frame = Project::getInstance().getFrame(num);
    if( frame->getFrametype() == KEYFRAME ){
        //load body joints
        auto bodyJoints = frame->getSkeleton()
                .getJointTree();
        auto it = bodyJoints.begin();
        while( it != bodyJoints.end() ){
            BodyJoint joint = *it;
            BodyJointItem* newItem = new BodyJointItem();

            newItem->setId(joint.getLimbID());
            newItem->setPos(joint.getImageLocation().x,joint.getImageLocation().y);
            scene->addItem(newItem);
            ++it;
        }
        //load body parts
        auto bodyParts = frame->getSkeleton()
                .getPartTree();
        auto pit = bodyParts.begin();
        while( pit != bodyParts.end() ){
            BodyPart part = *pit;
            BodyJointItem* child =
                    Utility::getJointItemById(scene->items(),part.getChildJoint());
            BodyJointItem* parent =
                    Utility::getJointItemById(scene->items(),part.getParentJoint());
            if( !child || !parent ){
                qDebug() << "Error! Joints not exist!" << endl;
                return;
            }
            BodyPartItem* newItem = new BodyPartItem(child,parent);

            newItem->setId(part.getPartID());
            scene->addItem(newItem);
            ++pit;
        }
    }
}

//TODO: [L] Fix zoom in / zoom out
void FrameView2D::FrameGraphicsView::wheelEvent(QWheelEvent *event){
    // Scale the view / do the zoom
    double scaleFactor = 1.15;
    if(event->delta() > 0) {
        qreal factor = transform().scale(scaleFactor, scaleFactor)
                .mapRect(QRectF(0, 0, 1, 1))
                .width();
        if (factor < 0.07 || factor > 100) return;
        // Zoom in
        scale(scaleFactor, scaleFactor);
    } else {
        qreal factor = transform().scale(1.0/scaleFactor, 1.0/scaleFactor)
                .mapRect(QRectF(0, 0, 1, 1))
                .width();
        if (factor < 0.07 || factor > 100) return;
        // Zooming out
        scale(1.0 / scaleFactor, 1.0 / scaleFactor);
    }
    //update();
}


