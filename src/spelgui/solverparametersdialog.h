#ifndef SOLVERPARAMETERSDIALOG_H
#define SOLVERPARAMETERSDIALOG_H

// SPEL definitions
#include "predef.hpp"
#include <QDialog>

#include <GUISolveParameters.h>

namespace Ui {
  class SolverParametersDialog;
}

class SolverParametersDialog : public QDialog
{
  Q_OBJECT

public:
  explicit SolverParametersDialog(QWidget *parent = 0);
  ~SolverParametersDialog();
  void removeEmptyCells();
  void addParameter(std::pair<std::string, float> parameter);
  void addParameters(std::map<std::string, float> parameter);
  void setDefaultGUIParameters();
  std::map<std::string, float> ExtractParameters(std::string TabName, std::map<std::string, float> parameters);
  std::map<std::string, float> getAllParameters();
  void copyTableCells(std::string group);
  std::map<std::string, std::map<std::string, float>> GroupedParameters;
signals:
  void tabBarClicked_(QWidget *, int );
  void ParametersUpdated(std::map<std::string, float>);
  //void tabBarClicked_(QWidget *);

  private slots:
  void AddButton_Clicked();
  void deleteButton_Clicked();
  void tabWidget_Clicked(int n);
  void tabWidget2_Clicked(int n);
  void tabWidget3_Clicked(int n);
  void OnTabBarClicked(QWidget * currentTab, int n);
  void DialogAccepted();
  //void OnTabBarClicked(QWidget * currentTab);

private:
  Ui::SolverParametersDialog *ui;
  /*std::map<std::string, float> GlobalParameters;
  std::map<std::string, float> SURFDetectorParameters;
  std::map<std::string, float> HOGDetectorParameters;
  std::map<std::string, float> CHDetectorParameters;
  std::map<std::string, float> nskpParameters;
  std::map<std::string, float> tlpsParameters;*/
 
  std::vector<std::string> GroupsNames;
  std::string previousGroup;
  std::string currentGroup;
};

#endif // SOLVERPARAMETERSDIALOG_H
