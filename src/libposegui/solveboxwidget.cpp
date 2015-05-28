#include "solveboxwidget.h"

#include "project.h"
#include <QtConcurrent/QtConcurrent>

using posegui::Project;
//PUBLIC

SolveBoxWidget::SolveBoxWidget(QWidget *parent) :
    QGroupBox(parent)
{
    //view
    interpolator = new QPushButton(this);
    interpolator->setIcon(QPixmap(QString(":/root/resources/icons/interpolation.png")));
    interpolator->setToolTip("Interpolate intermediate frames");
    interpolator->setEnabled(false);
    //TODO: add keyframe change state event

    solver = new QPushButton(this);
    solver->setIcon(QPixmap(QString(":/root/resources/icons/solving.png")));
    solver->setToolTip("Solve");
    //layouts
    MainLayout = new FlowLayout();
    MainLayout->addWidget(interpolator);
    MainLayout->addWidget(solver);
    //settings
    this->setEnabled(false);
    this->setLayout(MainLayout);
    //connect
    QObject::connect(interpolator,&QPushButton::clicked,
                     this,&SolveBoxWidget::interpolatorClicked);
    QObject::connect(solver,&QPushButton::clicked,
                     this,&SolveBoxWidget::solverClicked);
}

SolveBoxWidget::~SolveBoxWidget(){
    delete interpolator;
    delete solver;
    delete MainLayout;
}

void SolveBoxWidget::loadProjectEvent(){
    this->setEnabled(true);
}

void SolveBoxWidget::closeProjectEvent(){
    this->setEnabled(false);
}

void SolveBoxWidget::keyframeUpdatedEvent(){
    interpolator->setEnabled(true);
}

void SolveBoxWidget::solveFinishedEvent(){
    solver->setEnabled(true);
}

//PRIVATE

void SolveBoxWidget::interpolatorClicked(){
    Project::getInstance().interpolateFrames();
    interpolator->setEnabled(false);
}

void SolveBoxWidget::solverClicked(){
    QFuture<void> some = QtConcurrent::run(&Project::getInstance(),&Project::solveFrames);
    Project::getInstance().futureWatcher.setFuture(some);
    solver->setEnabled(false);
}
