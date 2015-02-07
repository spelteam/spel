#ifndef FRAMEBOXWIDGET_H
#define FRAMEBOXWIDGET_H

#include <QGroupBox>
#include <QHBoxLayout>
#include <QPushButton>
#include <QSlider>

class FrameBoxWidget : public QGroupBox
{
    Q_OBJECT
public:
    explicit FrameBoxWidget(QWidget *parent = 0);
    ~FrameBoxWidget();
signals:

public slots:

private slots:
    void changeViewerClicked();

private:
    //temp
    int state = 0;
    //layouts
    QHBoxLayout *MainLayout;
public:
    //view
    QPushButton *changeViewer;
    QPushButton *maskEditor;
    QSlider *itemSkaler;
    QSlider *maskViewer;

};

#endif // FRAMEBOXWIDGET_H
