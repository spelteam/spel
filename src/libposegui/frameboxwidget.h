#ifndef FRAMEBOXWIDGET_H
#define FRAMEBOXWIDGET_H

#include <QGroupBox>
#include <QHBoxLayout>
#include <QPushButton>
#include <QSlider>


class FrameTableWidget;

class FrameBoxWidget : public QGroupBox
{
    Q_OBJECT
public:
    explicit FrameBoxWidget( QWidget *parent = 0 );
    ~FrameBoxWidget();
signals:
    void changeFrametype( int frameNum );

public slots:
    void pickFrameEvent( int num );
    void loadProjectEvent();
    void closeProjectEvent();
private slots:
    void changeViewerClicked();
    void frametypeSelectorClicked();
private:
    //temp
    int state = 0;
    //layouts
    QHBoxLayout *MainLayout;
    //type of current frame
    bool isKeyframe;
    //number of current frame
    int num;
public:
    //view
    QPushButton *changeViewer;
    QPushButton *frametypeSelector;
    QPushButton *maskEditor;
    QSlider *itemSkaler;
    QSlider *maskViewer;

};

#endif // FRAMEBOXWIDGET_H
