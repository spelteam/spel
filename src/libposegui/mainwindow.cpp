#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QtConcurrent/QtConcurrent>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QMessageBox>
#include <QFileDialog>
#include <QInputDialog>
#include <QProgressBar>

#include "frametablewidget.h"
#include "toolboxwidget.h"
#include "solveboxwidget.h"
#include "frameboxwidget.h"
#include "frameview2d.h"

#include "project.h"
#include "utility.h"
#include "exceptions.h"

using namespace posegui;
//PUBLIC

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    //view
    ui->setupUi(this);
    framesView = new FrameTableWidget();
    //editTools = new ToolBoxWidget();
    solveTools = new SolveBoxWidget();
    frameTools = new FrameBoxWidget(framesView);
    currFrame = new FrameView2D();
    //utilites
    progressBar = new QProgressBar();
    progressBar->setVisible(false);
    progressBar->setTextVisible(true);
    progressBar->setMinimum(0);
    progressBar->setMaximum(100);
    //layouts
    FrameLayout = new QVBoxLayout;
    FrameLayout->addWidget(frameTools,1);
    FrameLayout->addWidget(currFrame->view,10);

    ToolLayout = new QHBoxLayout;
    //ToolLayout->addWidget(editTools,1);
    ToolLayout->addLayout(FrameLayout,8);
    ToolLayout->addWidget(solveTools,1);

    MainLayout = new QVBoxLayout;
    MainLayout->addLayout(ToolLayout, 8);
    MainLayout->addWidget(framesView,3);
    MainLayout->addWidget(progressBar,1);
    ui->centralWidget->setLayout(MainLayout);
    //set styles for group boxes
    this->setStyleSheet( posegui::Utility::fileToString(":/root/resources/stylesheets/Toolbox.qss") );
    //connect
    Project::getInstance();//connect events at Project constructor
    //loading
    QObject::connect(&Project::getInstance(),&Project::load,framesView,&FrameTableWidget::loadProjectEvent);
    QObject::connect(&Project::getInstance(),&Project::load, currFrame, &FrameView2D::loadProjectEvent );
    QObject::connect(&Project::getInstance(),&Project::load, frameTools, &FrameBoxWidget::loadProjectEvent );
    QObject::connect(&Project::getInstance(),&Project::load,solveTools, &SolveBoxWidget::loadProjectEvent);
    //closing
    QObject::connect(&Project::getInstance(),&Project::close,framesView,&FrameTableWidget::closeProjectEvent);
    QObject::connect(&Project::getInstance(),&Project::close,currFrame,&FrameView2D::closeProjectEvent);
    QObject::connect(&Project::getInstance(),&Project::close,frameTools,&FrameBoxWidget::closeProjectEvent);
    QObject::connect(&Project::getInstance(),&Project::close,solveTools, &SolveBoxWidget::closeProjectEvent);
    //scaling
    QObject::connect(frameTools->itemSkaler, &QSlider::valueChanged, currFrame, &FrameView2D::scaleItemsEvent);
    //mask opacity
    QObject::connect(frameTools->maskViewer, &QSlider::valueChanged, currFrame, &FrameView2D::changeMaskOpacityEvent);
    //picking frame
    QObject::connect(framesView, &FrameTableWidget::pickFrame, currFrame, &FrameView2D::pickFrameEvent);
    QObject::connect(framesView, &FrameTableWidget::pickFrame, frameTools, &FrameBoxWidget::pickFrameEvent);
    //change frametype
    QObject::connect(frameTools,&FrameBoxWidget::changeFrametype,
                     framesView,&FrameTableWidget::changeFrametypeEvent);
    //interpolation events
    QObject::connect(&Project::getInstance(),&Project::keyframeUpdated,
                     solveTools,&SolveBoxWidget::keyframeUpdatedEvent);
    //task progress
    QObject::connect(&watcher,&QFutureWatcher<void>::started,
                     progressBar, &QProgressBar::show);
    QObject::connect(&watcher,&QFutureWatcher<void>::finished,
                     progressBar, &QProgressBar::hide);
    //QObject::connect(&Project::getInstance().futureWatcher,&QFutureWatcher<void>::finished,
    //                 solveTools,&SolveBoxWidget::solveFinishedEvent);
    //TODO:[!] update solve status !!!!
    QObject::connect(&watcher,&QFutureWatcher<void>::progressValueChanged,
                     progressBar, &QProgressBar::setValue);
    QObject::connect(&watcher,&QFutureWatcher<void>::progressTextChanged,
                     progressBar, &QProgressBar::setFormat);

}

MainWindow::~MainWindow()
{
    delete ui;
    delete framesView;
    //delete editTools;
    delete solveTools;
    delete frameTools;
    delete  currFrame;
}

//PROTECTED

void MainWindow::closeEvent(QCloseEvent *){
    Project::getInstance().close();
}

//PRIVATE

void MainWindow::on_actionClose_triggered()
{
    ui->statusBar->showMessage("Closing project");
    Project::getInstance().close();
    ui->statusBar->showMessage("Project was closed");
}

#include <QDebug>
#include <QTime>
void MainWindow::on_actionOpen_triggered()
{
   //get filename from OpenFileDialog
   QString projectFilename = QFileDialog::getOpenFileName(
         this, //parent
         "Open project", //caption
         "", //start directory
         "Project files (*.xml)" //filter files
   );

	//linux testing path
    //QString projectFilename =
    //        "/files/Documents/Work/Libpose/src/utils/detectorTests/testdata1/trijumpSD_new.xml";
    //windows testing path
    //QString projectFilename =
    //       "D:/Documents/Work/Libpose/src/tests/testdata1/trijumpSD_new.xml";
    //try to open project
    ui->statusBar->showMessage("Loading project");
    try{
        QTime timer;
        timer.start();
        Project& project = Project::getInstance();
        QFuture<void> future =
                QtConcurrent::run(&project,&Project::open,projectFilename,&watcher);
        watcher.setFuture(future);
        watcher.waitForFinished();
        int elapsed = timer.elapsed();
        //load project to GUI
        Project::getInstance().load();
        ui->statusBar->showMessage("Project was loaded: "+QString::number(elapsed));
    } catch( const QException& e ){
        QMessageBox messageBox;
        messageBox.setWindowTitle(this->windowTitle());
        messageBox.setText(QString(e.what()));
        messageBox.setIcon(QMessageBox::Critical);
        messageBox.exec();
    }
}

void MainWindow::on_actionSave_triggered()
{
    try{
        QTime timer;
        timer.start();
        Project& project = Project::getInstance();
        QFuture<void> future =
                QtConcurrent::run(&project,&Project::save,&watcher);
        watcher.setFuture(future);
        watcher.waitForFinished();
        int elapsed = timer.elapsed();
        ui->statusBar->showMessage("Project was saved: "+QString::number(elapsed));
    } catch( const QException& e ){
        QMessageBox messageBox;
        messageBox.setWindowTitle(this->windowTitle());
        messageBox.setText(QString(e.what()));
        messageBox.setIcon(QMessageBox::Critical);
        messageBox.exec();
    }
}

void MainWindow::on_actionSave_as_triggered()
{
    try{
        QTime timer;
        timer.start();
        Project& project = Project::getInstance();
        QString filename = QInputDialog::getText(
            this, //parent
            "Save project", //title
            "Enter filename:" //label
        );
        QFuture<void> future =
                QtConcurrent::run(&project,&Project::save,filename,&watcher);
        watcher.setFuture(future);
        watcher.waitForFinished();
        int elapsed = timer.elapsed();
        ui->statusBar->showMessage("Project was saved: "+QString::number(elapsed));
    } catch( const QException& e ){
        QMessageBox messageBox;
        messageBox.setWindowTitle(this->windowTitle());
        messageBox.setText(QString(e.what()));
        messageBox.setIcon(QMessageBox::Critical);
        messageBox.exec();
    }
}
