// SPEL definitions
#include "predef.hpp"

#include "solverparametersdialog.h"
#include "ui_solverparametersdialog.h"

#include "utility.h"

SolverParametersDialog::SolverParametersDialog(QWidget *parent) :
QDialog(parent),
ui(new Ui::SolverParametersDialog)
{
  ui->setupUi(this);
  //init solver tab with NSKP params
  ui->grbTLPSParams->setVisible(false);
  ui->grbNSKPParams->setVisible(true);
  //set styles for group boxes
  //this->setStyleSheet( posegui::Utility::fileToString(":/root/resources/stylesheets/GroupBox2.qss") );
}

SolverParametersDialog::~SolverParametersDialog()
{
  delete ui;
}
//TODO:[!]Fix some params from Double to Int values
void SolverParametersDialog::on_cbbChooseSolver_currentIndexChanged(int index)
{
  //swap solver params(NSKP & TLPS)
  switch (index) {
  case 0: //NSKP
    ui->grbTLPSParams->setVisible(false);
    ui->grbNSKPParams->setVisible(true);
    break;
  case 1: //TLPS
    ui->grbTLPSParams->setVisible(true);
    ui->grbNSKPParams->setVisible(false);
    break;
  default:
    break;
  };
}
