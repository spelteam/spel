#include "frameview2d.h"

FrameView2D::FrameView2D(QWidget *parent) :
    QWidget(parent)
{
    scene = new QGraphicsScene;
    view = new QGraphicsView(scene,parent);
}

FrameView2D::~FrameView2D(){
    delete view;
    delete scene;
}
