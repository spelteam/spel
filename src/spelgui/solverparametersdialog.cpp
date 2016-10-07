// SPEL definitions
#include "predef.hpp"

#include "solverparametersdialog.h"
#include "ui_solverparametersdialog.h"
#include "spelParameters.hpp"
#include "GUISolveParameters.h"

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
  ui->tableWidget->setSelectionMode(QAbstractItemView::NoSelection);
  ui->tableWidget->setCurrentCell(0, 0);

  ui->tabWidget->setCurrentWidget(ui->tab_3);
  ui->label->setText(ui->tabWidget->tabBar()->tabText(0));
  currentGroup = ui->tabWidget->tabBar()->tabText(0).toStdString();
  previousGroup = currentGroup;
  
  std::vector<QString> QGroupsNames;
  QGroupsNames = { ui->tabWidget->tabText(0), ui->tabWidget->tabText(1), ui->tabWidget->tabText(2),
      ui->tabWidget_2->tabText(0),  ui->tabWidget_2->tabText(1),
      ui->tabWidget_3->tabText(0), ui->tabWidget_3->tabText(1), ui->tabWidget_3->tabText(2) };
  for (int i = 0; i < QGroupsNames.size(); i++)
    GroupsNames.push_back(QGroupsNames[i].toStdString());


  //Testing
  //ui->tableWidget->sortByColumn(0, Qt::SortOrder::AscendingOrder);
  /*ui->tableWidget->setCurrentCell(1, 2);
  ui->tableWidget->cellPressed(1, 2);*/
  /*addParameter(std::pair<std::string, float>("X", 20));
  std::map<std::string, float> P = { std::pair<std::string, float>("1",10), std::pair<std::string, float>("2", 20), 
                                     std::pair<std::string, float>("3", 30), std::pair<std::string, float>("4", 40) };
  addParameters(P);*/

  //Connect
  QObject::connect(ui->pushButton, &QPushButton::clicked, this, &SolverParametersDialog::AddButton_Clicked);
  QObject::connect(ui->pushButton_2, &QPushButton::clicked, this, &SolverParametersDialog::deleteButton_Clicked);
  QObject::connect(this, &SolverParametersDialog::tabBarClicked_, this, &SolverParametersDialog::OnTabBarClicked);

  QObject::connect(ui->tabWidget, &QTabWidget::tabBarClicked, this, &SolverParametersDialog::tabWidget_Clicked);
  QObject::connect(ui->tabWidget_2, &QTabWidget::tabBarClicked, this, &SolverParametersDialog::tabWidget2_Clicked);
  QObject::connect(ui->tabWidget_3, &QTabWidget::tabBarClicked, this, &SolverParametersDialog::tabWidget3_Clicked);

  //QObject::connect(ui->buttonBox, &QDialogButtonBox::accepted, this, &SolverParametersDialog::DialogAccepted);

  setDefaultGUIParameters();
  /*if(ui->tableWidget->rowCount() > 0)*/
  removeEmptyCells();
  emit tabBarClicked_(ui->tabWidget, 0);

  //init solver tab with NSKP params
  //set styles for group boxes
  //this->setStyleSheet( posegui::Utility::fileToString(":/root/resources/stylesheets/GroupBox2.qss") );
}

void SolverParametersDialog::setDefaultGUIParameters()
{
  std::map<std::string, float> temp;
  /*std::vector<QWidget*> TabBars= { ui->tab_3, ui->tab_2, ui->tab, ui->tab_4, ui->tab_5, ui->tab_6, ui->tab_7, ui->tab_8 };
  std::vector<QWidget*> TabBars = { ui->tab_3, ui->tab_2, ui->tab, ui->tab_4, ui->tab_5, ui->tab_6, ui->tab_7, ui->tab_8 };
  for(unsigned int i = 0; i < TabBars.size(); i++)
  {
    std::string GroupName = TabBars[i]-->tabText(0).toStdString();
    groupsNames.push_back(GroupName);
  }*/

  GroupedParameters = setDefaultParameters(GroupsNames);

  //Disabling the SURF detector
  if (GroupedParameters.find("Global") != GroupedParameters.end())
    if (GroupedParameters["Global"].find("useSURFdet") != GroupedParameters["Global"].end())
      GroupedParameters["Global"]["useSURFdet"] = 0.0f;

}

std::map<std::string, float> SolverParametersDialog::ExtractParameters(std::string TabName, std::map<std::string, float> parameters)
{
  std::map<std::string, std::map<std::string, float>>::iterator g;
  if(GroupedParameters.find(TabName) != GroupedParameters.end())
  {
    std::map<std::string, float>::iterator p;
    for (p = GroupedParameters[TabName].begin(); p != GroupedParameters[TabName].end(); p++)
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

void SolverParametersDialog::addParameter(std::pair<std::string, float> parameter)
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

void SolverParametersDialog::addParameters(std::map<std::string, float> parameters)
{
  for(std::map<std::string, float>::iterator p = parameters.begin(); p != parameters.end(); ++p)
    addParameter(*p);
}

void SolverParametersDialog::AddButton_Clicked()
{
  ui->tableWidget->setSortingEnabled(false);
  int n = ui->tableWidget->rowCount();
  ui->tableWidget->setRowCount(n + 1); 
  QTableWidgetItem* temp = new QTableWidgetItem(QString::fromStdString(""));
  ui->tableWidget->setItem(n, 0, temp);
  temp->setSelected(true);
  ui->tableWidget->editItem(temp);
  //ui->tableWidget->setSortingEnabled(true);
  getAllParameters();
}

void SolverParametersDialog::deleteButton_Clicked()
{
  ui->tableWidget->setSortingEnabled(false);
  int n = ui->tableWidget->currentRow();

  std::string name;
  if (n >= 0 && n < ui->tableWidget->rowCount())
  {
    name = ui->tableWidget->item(n, 0)->text().toStdString();
    ui->tableWidget->removeRow(n);
  }

  if (GroupedParameters.find(currentGroup) != GroupedParameters.end())
  {
    if (GroupedParameters.at(currentGroup).find(name) != GroupedParameters.at(currentGroup).end())
      GroupedParameters[currentGroup].erase(name);
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
  ui->tableWidget->setSortingEnabled(false);
  removeEmptyCells();
  for (int i = 0; i < ui->tableWidget->rowCount(); i++)
  {
    if(ui->tableWidget->item(i, 0) != 0)
    {
      if (ui->tableWidget->item(i, 0)->text() != "")
      {
        std::string name = ui->tableWidget->item(i, 0)->text().toStdString();
        float value = ui->tableWidget->item(i, 1)->text().toFloat();
        if (GroupedParameters.find(group) == GroupedParameters.end())
          GroupedParameters.emplace(group, std::map<std::string, float>());
        if (GroupedParameters.at(group).find(name) == GroupedParameters.at(group).end())
          GroupedParameters[group].emplace(std::pair<std::string, float>(name, value));
        else
          GroupedParameters[group][name] = value;
      }
    }
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
    addParameters(parameters);
    //ui->tableWidget->setSortingEnabled(true);
    /*QString temp;
    temp.setNum(GroupedParameters.at(group).size());
    ui->label->setText(temp);*/
  }
  ui->tableWidget->setSortingEnabled(true);
  ui->tableWidget->setSortingEnabled(false);
  ui->groupBox->show();
}

std::map<std::string, float> SolverParametersDialog::getAllParameters()
{
	//copyTableCells(previousGroup);
  std::map<std::string, float> parameters;
  for (int i = GroupsNames.size() - 1; i > -1; i--)
    parameters = ExtractParameters(GroupsNames[i], parameters);

  return parameters;
}


void SolverParametersDialog::DialogAccepted()
{
  emit  ParametersUpdated(getAllParameters());
}


SolverParametersDialog::~SolverParametersDialog()
{
  delete ui;
}

