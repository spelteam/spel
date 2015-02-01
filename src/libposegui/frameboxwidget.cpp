#include "frameboxwidget.h"

FrameBoxWidget::FrameBoxWidget(QWidget *parent) :
    QGroupBox(parent)
{
    //view
    changeViewer = new QPushButton(this);
    changeViewer->setIcon(QPixmap(QString(":/root/resources/icons/key_free.png")));
    changeViewer->setToolTip("Change view to 2D/3D");

    maskEditor = new QPushButton(this);
    maskEditor->setIcon(QPixmap(QString(":/root/resources/icons/big_mask.png")));
    maskEditor->setToolTip("Create/Edit mask");
    //layouts
    MainLayout = new QHBoxLayout;
    MainLayout->addWidget(changeViewer);
    MainLayout->addWidget(maskEditor);
    MainLayout->addStretch(0);
    //settings
    this->setLayout(MainLayout);
    //connect
    QObject::connect(changeViewer,&QPushButton::clicked,this,&FrameBoxWidget::changeViewerClicked);
}

FrameBoxWidget::~FrameBoxWidget(){
    delete changeViewer;
    delete MainLayout;
}

//private

void FrameBoxWidget::changeViewerClicked(){
    //temp
    if( state == 0 ){
        state = 1;
        changeViewer->setIcon(QPixmap(QString(":/root/resources/icons/key_lock.png")));
        return;
    }
    if( state == 1 ){
        state = 0;
        changeViewer->setIcon(QPixmap(QString(":/root/resources/icons/key_free.png")));
        return;
    }
}
