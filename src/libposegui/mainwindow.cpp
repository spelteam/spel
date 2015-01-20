#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QHBoxLayout>
#include <QVBoxLayout>

#include "frametablewidget.h"
#include "toolboxwidget.h"
#include "solveboxwidget.h"
#include "frameboxwidget.h"
#include "frameview2d.h"

#include "project.h"

#include <QFile>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    //view
    ui->setupUi(this);
    framesView = new FrameTableWidget();
    editTools = new ToolBoxWidget();
    solveTools = new SolveBoxWidget();
    frameTools = new FrameBoxWidget();
    currFrame = new FrameView2D();
    //layouts
    FrameLayout = new QVBoxLayout;
    FrameLayout->addWidget(frameTools,1);
    FrameLayout->addWidget(currFrame->view,10);

    ToolLayout = new QHBoxLayout;
    ToolLayout->addWidget(editTools,1);
    ToolLayout->addLayout(FrameLayout,8);
    ToolLayout->addWidget(solveTools,1);

    MainLayout = new QVBoxLayout;
    MainLayout->addLayout(ToolLayout, 5);
    MainLayout->addWidget(framesView,1);
    ui->centralWidget->setLayout(MainLayout);
    //model
    currProject = new Project(this);
    //set styles for group boxes
    QFile styleFile(":/root/resources/stylesheets/QGroupBox.qss");
    styleFile.open( QFile::ReadOnly );
    this->setStyleSheet( QString(styleFile.readAll()) );
    styleFile.close();
    //connect
    QObject::connect(currProject,&Project::open,framesView,&FrameTableWidget::openProjectEvent);
    QObject::connect(currProject,&Project::close,framesView,&FrameTableWidget::closeProjectEvent);
}

MainWindow::~MainWindow()
{
    delete ui;
    delete framesView;
    delete editTools;
    delete solveTools;
    delete frameTools;
    delete  currFrame;

    delete currProject;
}

void MainWindow::on_actionClose_triggered()
{
     currProject->close();
}
