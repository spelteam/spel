#ifndef SOLVERPARAMETERSDIALOG_H
#define SOLVERPARAMETERSDIALOG_H

// SPEL definitions
#include "predef.hpp"
#include <QDialog>
#include <QTreeWidget>
#include <tree.hh>
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
  void putParameter(std::pair<std::string, float> parameter);
  void putParameters(std::map<std::string, float> parameters);

  void setDefaultParameters();
  //void setParameters(std::map<std::string, float> parameters);
  std::map<std::string, std::map<std::string, float>> GroupeParameters(std::map<std::string, float> parameters);
  std::map<std::string, std::map<std::string, float>> RegroupeParameters();
  void clearParameters();
  std::map<std::string, float> getParameters(std::string groupName, std::map<std::string, float> parameters);
  std::map<std::string, float> getUngroupedParameters();
  std::map<std::string, std::map<std::string, float>> getGroupedParameters();
  void setGroupedParameters(std::map<std::string, std::map<std::string, float>> G);
  void copyTableCells(std::string group);

signals:
  void tabBarClicked_(QWidget *, int );
  //void ParametersUpdated(std::map<std::string, float>);
  void ParametersUpdated(std::map<std::string, std::map<std::string, float>>);
  //void tabBarClicked_(QWidget *);

  private slots:
  void AddButton_Clicked();
  void RemoveButton_Clicked();
  void tabWidget_Clicked(int n);
  void tabWidget2_Clicked(int n);
  void tabWidget3_Clicked(int n);
  void OnTabBarClicked(QWidget * currentTab, int n);
  void DialogAccepted();
  void DefaultButton_clicked();
  void Apply();
  //void OnTabBarClicked(QWidget * currentTab);

private:
  Ui::SolverParametersDialog *ui;
  /*std::map<std::string, float> GlobalParameters;
  std::map<std::string, float> SURFDetectorParameters;
  std::map<std::string, float> HOGDetectorParameters;
  std::map<std::string, float> CHDetectorParameters;
  std::map<std::string, float> nskpParameters;
  std::map<std::string, float> tlpsParameters;*/
  std::map<std::string, std::map<std::string, float>> GroupedParameters;
  std::vector<std::string> GroupsNames;
  std::string previousGroup;
  std::string currentGroup;
};

#endif // SOLVERPARAMETERSDIALOG_H
