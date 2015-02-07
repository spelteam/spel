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
    MainLayout->addLayout(ToolLayout, 9);
    MainLayout->addWidget(framesView,3);
    ui->centralWidget->setLayout(MainLayout);
    //set styles for group boxes
    this->setStyleSheet( Utility::fileToString(":/root/resources/stylesheets/Toolbox.qss") );
    //connect
    //loading
    QObject::connect(&Project::getInstance(),&Project::load,framesView,&FrameTableWidget::loadProjectEvent);
    QObject::connect(&Project::getInstance(),&Project::load, currFrame, &FrameView2D::loadProjectEvent );
    //closing
    QObject::connect(&Project::getInstance(),&Project::close,framesView,&FrameTableWidget::closeProjectEvent);
    QObject::connect(&Project::getInstance(),&Project::close,currFrame,&FrameView2D::closeProjectEvent);
    //scaling
    QObject::connect(frameTools->itemSkaler, &QSlider::valueChanged, currFrame, &FrameView2D::scaleItemsEvent);
    //mask opacity
    QObject::connect(frameTools->maskViewer, &QSlider::valueChanged, currFrame, &FrameView2D::changeMaskOpacityEvent);
    //picking frame
    QObject::connect(framesView, &FrameTableWidget::cellClicked,currFrame,&FrameView2D::pickFrameEvent);
    QObject::connect(framesView, &FrameTableWidget::cellActivated,currFrame,&FrameView2D::pickFrameEvent);
    QObject::connect(framesView, &FrameTableWidget::cellEntered,currFrame,&FrameView2D::pickFrameEvent);
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
    ui->statusBar->showMessage("Closing project");
    Project::getInstance().close();
    ui->statusBar->showMessage("Project was closed");
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

    Project::ErrorCode errCode = Project::getInstance().open(
       projectFilename
    );
    if( errCode != Project::ErrorCode::SUCCESS ){
        QMessageBox messageBox;
        messageBox.setWindowTitle(this->windowTitle());
        messageBox.setText(Project::getInstance().getLastError());
        messageBox.setIcon(QMessageBox::Critical);
        messageBox.exec();
    } else{
        QMessageBox messageBox;
        messageBox.setWindowTitle(this->windowTitle());
        messageBox.setText("All ok");
        messageBox.setIcon(QMessageBox::Information);
        messageBox.exec();
        //load project to GUI
       Project::getInstance().load();
       ui->statusBar->showMessage("Project was loaded");
    }
}
