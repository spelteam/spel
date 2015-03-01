#ifndef FRAMEVIEW2D_H
#define FRAMEVIEW2D_H

#include <QWidget>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QList>

class BodyJointItem;

class FrameView2D : public QWidget
{
    Q_OBJECT
private:
    class FrameGraphicsView : public QGraphicsView
    {
    public:
        FrameGraphicsView(QWidget* parent = 0):
            QGraphicsView(parent){
            setDragMode(DragMode::ScrollHandDrag);
            setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
            setRenderHint(QPainter::Antialiasing);
            setViewportUpdateMode(BoundingRectViewportUpdate);
            setCacheMode(CacheBackground);
        }
        FrameGraphicsView(QGraphicsScene* scene, QWidget* parent = 0):
            QGraphicsView(scene,parent){
            setDragMode(DragMode::ScrollHandDrag);
            setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
            setRenderHint(QPainter::Antialiasing);
            setViewportUpdateMode(BoundingRectViewportUpdate);
        }
    protected:
        void wheelEvent(QWheelEvent* event) override;
    };
public:
    explicit FrameView2D(QWidget *parent = 0);
    ~FrameView2D();

signals:

public slots:
    void loadProjectEvent();
    void closeProjectEvent();
    void scaleItemsEvent( int value );
    void changeMaskOpacityEvent( int value );
    void pickFrameEvent(int, int col);

public:
    FrameGraphicsView *view;
    QGraphicsScene *scene;

private:
    void loadFrameImage( int num );
    void loadFrameJoints( int num );

private:
    QGraphicsPixmapItem* frameImage;
};
#endif // FRAMEVIEW2D_H
