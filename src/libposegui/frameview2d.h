#ifndef FRAMEVIEW2D_H
#define FRAMEVIEW2D_H

#include <QWidget>
#include <QGraphicsView>
#include <QGraphicsScene>

class FrameView2D : public QWidget
{
    Q_OBJECT
public:
    explicit FrameView2D(QWidget *parent = 0);
    ~FrameView2D();

signals:

public slots:

public:
    QGraphicsView *view;
    QGraphicsScene *scene;

};

#endif // FRAMEVIEW2D_H
