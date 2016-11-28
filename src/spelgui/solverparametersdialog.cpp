// SPEL definitions
#include "predef.hpp"

#include "solverparametersdialog.h"
#include "ui_solverparametersdialog.h"
#include "spelParameters.hpp"
#include "GUISolveParameters.h"
#include "tree.hh"


using namespace posegui;
using namespace std;

SolverParametersDialog::SolverParametersDialog(QWidget *parent) :
QDialog(parent),
ui(new Ui::SolverParametersDialog)
{
  ui->setupUi(this);

  ui->tableWidget->horizontalHeader()->resizeSection(0, 180);
  ui->tableWidget->horizontalHeader()->setMaximumSectionSize(220);
  ui->tableWidget->horizontalHeader()->setSortIndicator(0, Qt::SortOrder::AscendingOrder);
  ui->tableWidget->setFixedWidth(311);
  ui->tableWidget->setSelectionMode(QAbstractItemView::NoSelection);
  ui->tableWidget->setCurrentCell(0, 0);

  ui->tabWidget->setCurrentWidget(ui->tab_3);
  ui->label->setText(ui->tabWidget->tabBar()->tabText(0));
  currentGroup = ui->tabWidget->tabBar()->tabText(0).toStdString();
  previousGroup = currentGroup;
  
  std::vector<QString> QGroupsNames;
  QGroupsNames = { ui->tabWidget->tabText(0), /*ui->tabWidget->tabText(1), ui->tabWidget->tabText(2),*/
      ui->tabWidget_2->tabText(0),  ui->tabWidget_2->tabText(1),
      ui->tabWidget_3->tabText(0), ui->tabWidget_3->tabText(1), ui->tabWidget_3->tabText(2) };
  for (int i = 0; i < QGroupsNames.size(); i++)
    GroupsNames.push_back(QGroupsNames[i].toStdString());

  //Connect
  QObject::connect(ui->AddButton, &QPushButton::clicked, this, &SolverParametersDialog::AddButton_Clicked);
  QObject::connect(ui->RemoveButton, &QPushButton::clicked, this, &SolverParametersDialog::RemoveButton_Clicked);
  QObject::connect(this, &SolverParametersDialog::tabBarClicked_, this, &SolverParametersDialog::OnTabBarClicked);

  QObject::connect(ui->tabWidget, &QTabWidget::tabBarClicked, this, &SolverParametersDialog::tabWidget_Clicked);
  QObject::connect(ui->tabWidget_2, &QTabWidget::tabBarClicked, this, &SolverParametersDialog::tabWidget2_Clicked);
  QObject::connect(ui->tabWidget_3, &QTabWidget::tabBarClicked, this, &SolverParametersDialog::tabWidget3_Clicked);

  QObject::connect(ui->DefaultButton, &QPushButton::clicked, this, &SolverParametersDialog::DefaultButton_clicked);
  QObject::connect(ui->ApplyButton, &QPushButton::clicked, this, &SolverParametersDialog::Apply);
  //QObject::connect(ui->treeWidget, &QTreeWidget::clicked, this, &SolverParametersDialog::Edit);

  setDefaultParameters();
  if(ui->tableWidget->rowCount() > 0)
    removeEmptyCells();
  putParameters(GroupedParameters.at(ui->tabWidget->tabText(0).toStdString()));
  currentGroup = ui->tabWidget->tabText(0).toStdString();
  emit tabBarClicked_(ui->tabWidget, 0);


  //init solver tab with NSKP params
  //set styles for group boxes
  //this->setStyleSheet( posegui::Utility::fileToString(":/root/resources/stylesheets/GroupBox2.qss") );
}

void SolverParametersDialog::setDefaultParameters()
{
  std::map<std::string, float> temp;

  GroupedParameters = DefaultParameters(GroupsNames);

  //Disabling the SURF detector
  if (GroupedParameters.find("Global") != GroupedParameters.end())
    if (GroupedParameters["Global"].find("useSURFdet") != GroupedParameters["Global"].end())
      GroupedParameters["Global"]["useSURFdet"] = 0.0f;

}
/*
void SolverParametersDialog::setParameters(std::map<std::string, float> params)
{
  std::map<std::string, float> P = params;
  if (P.size() > 0)
  {
    clearParameters();
    for (int i = ui->tableWidget->rowCount() - 1; i > -1; i--)
      ui->tableWidget->removeRow(i);
    setDefaultParameters();
    std::map<std::string, std::map<std::string, float>> temp;
    for (unsigned int i = 0; i < GroupsNames.size(); i++)
    {
      std::string group = GroupsNames[i];
      temp.emplace(std::pair<std::string, std::map<std::string, float>>(group, std::map<std::string, float>()));
      for (auto p = P.begin(); p != P.end(); p++)
      {
        std::string parameter = p->first;
        if (GroupedParameters[group].find(parameter) != GroupedParameters[group].end())
        {
          temp[group].emplace(*p);
          P.erase(parameter);
        }
      }
    }
    for (auto p = P.begin(); p != P.end(); p++)
    temp["Global"].emplace(*p);

    clearParameters();
    GroupedParameters = temp;
    if (ui->tableWidget->rowCount() > 0)
        removeEmptyCells();
    ui->tabWidget->setCurrentWidget(ui->tabWidget->widget(0));
    ui->groupBox->setParent(ui->tabWidget->widget(0));
    putParameters(GroupedParameters.at(ui->tabWidget->tabText(0).toStdString()));

    previousGroup = ui->tabWidget->tabText(0).toStdString();
    currentGroup = ui->tabWidget->tabText(0).toStdString();
    emit tabBarClicked_(ui->tabWidget, 0);
  }
}*/

void SolverParametersDialog::clearParameters()
{ 
  //std::map<std::string, std::map<std::string, float>>::iterator p;
  for(auto p = GroupedParameters.begin(); p != GroupedParameters.end(); p++)
    p->second.clear();
  GroupedParameters.clear();
}

std::map<std::string, float> SolverParametersDialog::getParameters(std::string groupName, std::map<std::string, float> parameters)
{
  std::map<std::string, std::map<std::string, float>>::iterator g;
  if(GroupedParameters.find(groupName) != GroupedParameters.end())
  {
    std::map<std::string, float>::iterator p;
    for (p = GroupedParameters[groupName].begin(); p != GroupedParameters[groupName].end(); p++)
      parameters.emplace(*p);
  }

  return parameters;
}


void SolverParametersDialog::removeEmptyCells()
{
  ui->tableWidget->setSortingEnabled(false);
  int n = ui->tableWidget->rowCount();
  for (int i = n - 1; i > 0; i--)
  {
    if(ui->tableWidget->item(i, 0) == 0) 
      ui->tableWidget->removeRow(i);
    else
    if (ui->tableWidget->item(i, 0)->text().isEmpty())
      ui->tableWidget->removeRow(i);
  }	  
  n = ui->tableWidget->rowCount();
  for (int i = n - 1; i > 0; i--)
  {
    if(ui->tableWidget->item(i, 1) == 0) 
      ui->tableWidget->removeRow(i);
    else
    if (ui->tableWidget->item(i, 1)->text().isEmpty())
      ui->tableWidget->removeRow(i);
  }	 
  //ui->tableWidget->setSortingEnabled(true);
}

void SolverParametersDialog::putParameter(std::pair<std::string, float> parameter)
{
  ui->tableWidget->setSortingEnabled(false);
  int n = ui->tableWidget->rowCount();
  ui->tableWidget->setRowCount(n + 1);
  QTableWidgetItem* temp = 0;
  temp = new QTableWidgetItem(QString::fromStdString(parameter.first));
  ui->tableWidget->setItem(n, 0, temp);
  temp = new QTableWidgetItem(QString::number(parameter.second));
  ui->tableWidget->setItem(n, 1, temp);
  //ui->tableWidget->setSortingEnabled(true);
}

void SolverParametersDialog::putParameters(std::map<std::string, float> parameters)
{
  for(std::map<std::string, float>::iterator p = parameters.begin(); p != parameters.end(); ++p)
    putParameter(*p);
}

void SolverParametersDialog::AddButton_Clicked()
{
  ui->tableWidget->setSortingEnabled(false);
  int n = ui->tableWidget->rowCount();
  ui->tableWidget->setRowCount(n + 1); 
  QTableWidgetItem* temp = new QTableWidgetItem(QString::fromStdString(""));
  ui->tableWidget->setItem(n, 0, temp);
  ui->tableWidget->setCurrentCell(n,0);
  ui->tableWidget->editItem(temp);
  //ui->tableWidget->setSortingEnabled(true);
}

void SolverParametersDialog::RemoveButton_Clicked()
{
  ui->tableWidget->setSortingEnabled(false);
  int n = ui->tableWidget->currentRow();

  std::string name;
  if (n >= 0 && n < ui->tableWidget->rowCount())
  {
    name = ui->tableWidget->item(n, 0)->text().toStdString();
    ui->tableWidget->removeRow(n);
  }

  //ui->tableWidget->setSortingEnabled(true);
}

void SolverParametersDialog::tabWidget_Clicked(int n)
{
  QTabWidget * p = 0;
  if (n == 0)  p = ui->tabWidget; 
  if (n == 1)  p = ui->tabWidget_2; 
  if (n == 2)  p = ui->tabWidget_3;
  p->setCurrentWidget(p->widget(0));
  emit tabBarClicked_(p, 0);
}
void SolverParametersDialog::tabWidget2_Clicked(int n)
{
  QTabWidget * p = ui->tabWidget_2;
  emit tabBarClicked_(p, n);
}
void SolverParametersDialog::tabWidget3_Clicked(int n)
{
  QTabWidget * p = ui->tabWidget_3;
  emit tabBarClicked_(p, n);
}

void SolverParametersDialog::copyTableCells(std::string group)
{
  if (GroupedParameters.find(group) == GroupedParameters.end())
    GroupedParameters.emplace(group, std::map<std::string, float>());
  //else
  GroupedParameters.at(group).clear();
  ui->tableWidget->setSortingEnabled(false);
  removeEmptyCells();
  for (int i = 0; i < ui->tableWidget->rowCount(); i++)
    if(ui->tableWidget->item(i, 0) != 0)
      if (ui->tableWidget->item(i, 0)->text() != "")
      {
        std::string name = ui->tableWidget->item(i, 0)->text().toStdString();
        float value = ui->tableWidget->item(i, 1)->text().toFloat();
        //if (GroupedParameters.at(group).find(name) == GroupedParameters.at(group).end())
          GroupedParameters[group].emplace(std::pair<std::string, float>(name, value));
        /*else
          GroupedParameters[group][name] = value;*/
      }
  //ui->tableWidget->setSortingEnabled(true);
}


void SolverParametersDialog::OnTabBarClicked(QWidget* p, int n)
{ 
  QTabWidget* P = static_cast<QTabWidget*>(p);
  currentGroup = P->tabBar()->tabText(n).toStdString();
  copyTableCells(previousGroup);
  ui->label->setText(QString::fromStdString(currentGroup));
  if (P->currentWidget() != ui->tab_2 && P->currentWidget() != ui->tab)
    previousGroup = currentGroup;

  ui->groupBox->setParent(P->widget(n));

  for(int i = ui->tableWidget->rowCount() - 1; i > -1; i--)
    ui->tableWidget->removeRow(i);

  //Reload parameters
  if(GroupedParameters.find(currentGroup) != GroupedParameters.end())
  {
    std::map<std::string, float> parameters;
    parameters = GroupedParameters.at(currentGroup);
    putParameters(parameters);
  }
  ui->tableWidget->setSortingEnabled(true);
  ui->tableWidget->setSortingEnabled(false);
  ui->groupBox->show();
}

void SolverParametersDialog::Apply()
{
  //emit ParametersUpdated(getUngroupedParameters());
  emit ParametersUpdated(getGroupedParameters());
}

std::map<std::string, float> SolverParametersDialog::getUngroupedParameters()
{
  copyTableCells(currentGroup);
  std::map<std::string, float> parameters;
  for (int i = GroupsNames.size() - 1; i > -1; i--)
    parameters = getParameters(GroupsNames[i], parameters);

  return parameters;
}

std::map<std::string, std::map<std::string, float>> SolverParametersDialog::getGroupedParameters()
{
  copyTableCells(currentGroup);
  return GroupedParameters;
}

void SolverParametersDialog::setGroupedParameters(std::map<std::string, std::map<std::string, float>> G)
{
  clearParameters();
  if (G.size() > 0)
  {
    for (int i = ui->tableWidget->rowCount() - 1; i > -1; i--)
      ui->tableWidget->removeRow(i);

    GroupedParameters = G;

    ui->tabWidget->setCurrentWidget(ui->tabWidget->widget(0));
    ui->groupBox->setParent(ui->tabWidget->widget(0));
    putParameters(GroupedParameters.at(ui->tabWidget->tabText(0).toStdString()));

    previousGroup = ui->tabWidget->tabText(0).toStdString();
    currentGroup = ui->tabWidget->tabText(0).toStdString();
    emit tabBarClicked_(ui->tabWidget, 0);
  }
  else
    setDefaultParameters();
}

void SolverParametersDialog::DialogAccepted()
{
  //emit ParametersUpdated(getUngroupedParameters());
  emit ParametersUpdated(GroupedParameters);
}


void SolverParametersDialog::DefaultButton_clicked()
{
  clearParameters();
  setDefaultParameters();
  if (ui->tableWidget->rowCount() > 0)
    removeEmptyCells();
  ui->tabWidget->setCurrentWidget(ui->tabWidget->widget(0));
  ui->groupBox->setParent(ui->tabWidget->widget(0));
  putParameters(GroupedParameters.at(ui->tabWidget->tabText(0).toStdString()));

  previousGroup = ui->tabWidget->tabText(0).toStdString();
  currentGroup = ui->tabWidget->tabText(0).toStdString();
  emit tabBarClicked_(ui->tabWidget, 0);
}

SolverParametersDialog::~SolverParametersDialog()
{
  delete ui;
}

