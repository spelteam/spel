#include "frameboxwidget.h"
#include "project.h"
#include "frametablewidget.h"

//TODO:[!] naked STRINGS!!!
FrameBoxWidget::FrameBoxWidget(QWidget *parent ) :
    QGroupBox(parent),
    isKeyframe(false),
    num(-1)
{
    //view
    changeViewer = new QPushButton(this);
    changeViewer->setIcon(QPixmap(QString(":/root/resources/icons/view2d.png")));
    changeViewer->setToolTip("Change view to 2D/3D");

    frametypeSelector = new QPushButton(this);
    frametypeSelector->setIcon(QPixmap(QString(":/root/resources/icons/key_free.png")));
    frametypeSelector->setToolTip("Make/Unmake keyframe");

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
    MainLayout = new QHBoxLayout();
    MainLayout->addWidget(changeViewer);
    MainLayout->addWidget(frametypeSelector);
    MainLayout->addWidget(maskEditor);
    MainLayout->addWidget(itemSkaler, 0, Qt::AlignLeft);
    MainLayout->addWidget(maskViewer, 0, Qt::AlignLeft);
    MainLayout->addStretch(0);
    //settings
    this->setEnabled(false);
    this->setLayout(MainLayout);
    //connect
    QObject::connect(changeViewer,&QPushButton::clicked,
                     this,&FrameBoxWidget::changeViewerClicked);
    QObject::connect(frametypeSelector,&QPushButton::clicked,
                     this,&FrameBoxWidget::frametypeSelectorClicked);
}

FrameBoxWidget::~FrameBoxWidget(){
    delete changeViewer;
    delete frametypeSelector;
    delete maskEditor;
    delete itemSkaler;
    delete maskViewer;
    delete MainLayout;
}

//PRIVATE

void FrameBoxWidget::changeViewerClicked(){
    //temp
    if( state == 0 ){
        state = 1;
        changeViewer->setIcon(QPixmap(QString(":/root/resources/icons/view3d.png")));
        return;
    }
    if( state == 1 ){
        state = 0;
        changeViewer->setIcon(QPixmap(QString(":/root/resources/icons/view2d.png")));
        return;
    }
}

void FrameBoxWidget::frametypeSelectorClicked(){
    //if no frame was selected, exit
    if( num == -1 ) return;
    if(isKeyframe){
        Project::getInstance().exchangeAtInterpolation(num);
    } else{
        Project::getInstance().exchangeAtKeyframe(num);
    }
    //update state of other widgets
    changeFrametype(num);
}

void FrameBoxWidget::pickFrameEvent(int num){
    //change view  of elements
    this->num = num; //update number of selected frame
    auto frame = Project::getInstance().getFrame(num);
    //check whether type of frame changed
    if( (frame->getFrametype() == KEYFRAME && !isKeyframe) ||
        (frame->getFrametype() != KEYFRAME && isKeyframe) )
    {
        if( frame->getFrametype() == KEYFRAME ){
            frametypeSelector->setIcon(QPixmap(QString(":/root/resources/icons/key_lock.png")));
            isKeyframe = true;
        } else {
            frametypeSelector->setIcon(QPixmap(QString(":/root/resources/icons/key_free.png")));
            isKeyframe = false;
        }
    }
}

void FrameBoxWidget::loadProjectEvent(){
    this->setEnabled(true);
}

void FrameBoxWidget::closeProjectEvent(){
    this->setEnabled(false);
    num = -1; //deselect any frame
}
