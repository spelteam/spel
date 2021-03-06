// SPEL definitions
#include "predef.hpp"

#include "solveboxwidget.h"

#include "solverparametersdialog.h"
#include "project.h"
#include <QtConcurrent/QtConcurrent>
#include <QErrorMessage>

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
  this->setEnabled(true);//TODO:[!]Set to false
  this->setLayout(MainLayout);
  //connect
  QObject::connect(interpolator, &QPushButton::clicked,
    this, &SolveBoxWidget::interpolatorClicked);
  QObject::connect(solver, &QPushButton::clicked,
    this, &SolveBoxWidget::solverClicked);
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

  SolverParametersDialog paramsDialog(this);
  /*auto P = Project::getInstance().getProjectParameters();
  paramsDialog.setParameters(P);*/
  auto P = Project::getInstance().getGroupedParameters();
  paramsDialog.setGroupedParameters(P);
  QObject::connect(&paramsDialog, &SolverParametersDialog::ParametersUpdated, &Project::getInstance(), &Project::onParametersUpdated);

  bool b = paramsDialog.exec();
  if (b)
  {
    if (Project::getInstance().getFrames().size() > 0)
    {
      /*std::map<std::string, float> params = paramsDialog.getUngroupedParameters();
      Project::getInstance().setProjectParameters(params);
      Project::getInstance().saveProgectParameters();*/

      std::map<std::string, std::map<std::string, float>>  G = paramsDialog.getGroupedParameters();
      Project::getInstance().setGroupedParameters(G);
      Project::getInstance().saveGroupedParameters();

      solver->setEnabled(false);
      emit startSolve();
      QFuture<void> some = QtConcurrent::run(&Project::getInstance(), &Project::solveFrames);
      Project::getInstance().futureWatcher.setFuture(some);
      //Project::getInstance().solveFrames();
    }
    else
      (new QErrorMessage(this))->showMessage("Solve error: Project not loaded");
  }
}

