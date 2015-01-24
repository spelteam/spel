#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QMessageBox>
#include <QFileDialog>

#include "frametablewidget.h"
#include "toolboxwidget.h"
#include "solveboxwidget.h"
#include "frameboxwidget.h"
#include "frameview2d.h"

#include "project.h"
#include "utility.h"

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
    //set styles for group boxes
    this->setStyleSheet( Utility::fileToString(":/root/resources/stylesheets/QGroupBox.qss") );
    //connect
    QObject::connect(&Project::getInstance(),&Project::load,framesView,&FrameTableWidget::loadProjectEvent);
    QObject::connect(&Project::getInstance(),&Project::close,framesView,&FrameTableWidget::closeProjectEvent);
}

MainWindow::~MainWindow()
{
    delete ui;
    delete framesView;
    delete editTools;
    delete solveTools;
    delete frameTools;
    delete  currFrame;
}

void MainWindow::on_actionClose_triggered()
{
    Project::getInstance().close();
}

#include <QDebug>
void MainWindow::on_actionOpen_triggered()
{
    //get filename from OpenFileDialog
   /* QString projectFilename = QFileDialog::getOpenFileName(
        this, //parent
        "Open project", //caption
        "", //start directory
        "Project files (*.xml)" //filter files
    );*/
    QString projectFilename =
            "/files/Documents/Work/Libpose/src/utils/detectorTests/testdata1/trijumpSD_new.xml";
    //try to open project
    ui->statusBar->showMessage("Loading project");
    QString errMessage;
    Project::ErrorCode errCode = Project::getInstance().open(
       projectFilename,
       &errMessage
    );
    if( errCode != Project::ErrorCode::SUCCESS ){
        QMessageBox messageBox;
        messageBox.setWindowTitle(this->windowTitle());
        messageBox.setText(errMessage);
        messageBox.setIcon(QMessageBox::Critical);
        messageBox.exec();
    } else{
        QMessageBox messageBox;
        messageBox.setWindowTitle(this->windowTitle());
        messageBox.setText("All ok");
        messageBox.setIcon(QMessageBox::Information);
        messageBox.exec();
    }
     //load project to GUI
    ui->statusBar->showMessage("Project was loaded");
}
