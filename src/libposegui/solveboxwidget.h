#ifndef SOLVEBOXWIDGET_H
#define SOLVEBOXWIDGET_H

#include <QGroupBox>
#include <QHBoxLayout>
#include <QPushButton>
#include "flowlayout.h"

class SolveBoxWidget : public QGroupBox
{
    Q_OBJECT
public:
    explicit SolveBoxWidget(QWidget *parent = 0);
    virtual ~SolveBoxWidget();
signals:

public slots:
    void loadProjectEvent();
    void closeProjectEvent();
    void keyframeUpdatedEvent();
private slots:
    void interpolatorClicked();
    void solverClicked();
private:
    //layouts
    FlowLayout *MainLayout;
public:
    //view
    QPushButton *interpolator;
    QPushButton *solver;
};

#endif // SOLVEBOXWIDGET_H
