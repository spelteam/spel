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

    //TODO:[T] change options
    itemSkaler = new QSlider(Qt::Orientation::Horizontal, this);
    itemSkaler->setToolTip("Scale body joints/parts");
    itemSkaler->setMinimum(10);
    itemSkaler->setMaximum(100);
    itemSkaler->setSingleStep(1);
    itemSkaler->setPageStep(1);
    //TODO:[T] chagne options
    maskViewer = new QSlider(Qt::Orientation::Horizontal, this);
    maskViewer->setToolTip("Set opacity of the mask");
    maskViewer->setMinimum(0);
    maskViewer->setMaximum(255);
    maskViewer->setSingleStep(1);
    maskViewer->setPageStep(1);
    //layouts
    MainLayout = new QHBoxLayout;
    MainLayout->addWidget(changeViewer);
    MainLayout->addWidget(maskEditor);
    MainLayout->addWidget(itemSkaler, 0, Qt::AlignLeft);
    MainLayout->addWidget(maskViewer, 0, Qt::AlignLeft);
    MainLayout->addStretch(0);
    //settings
    this->setLayout(MainLayout);
    //connect
    QObject::connect(changeViewer,&QPushButton::clicked,this,&FrameBoxWidget::changeViewerClicked);
}

FrameBoxWidget::~FrameBoxWidget(){
    delete changeViewer;
    delete maskEditor;
    delete itemSkaler;
    delete maskViewer;
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
