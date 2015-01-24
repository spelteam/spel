#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>


namespace Ui {
class MainWindow;
}
//layouts
class QHBoxLayout;
class QVBoxLayout;
//view
class FrameTableWidget;
class ToolBoxWidget;
class SolveBoxWidget;
class FrameBoxWidget;
class FrameView2D;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_actionClose_triggered();

    void on_actionOpen_triggered();

private:
    //layouts
    QVBoxLayout *FrameLayout;
    QHBoxLayout *ToolLayout;
    QVBoxLayout *MainLayout;
    //view
    Ui::MainWindow *ui;
    FrameTableWidget *framesView;
    ToolBoxWidget *editTools;
    SolveBoxWidget *solveTools;
    FrameBoxWidget *frameTools;
    FrameView2D *currFrame;
};

#endif // MAINWINDOW_H
