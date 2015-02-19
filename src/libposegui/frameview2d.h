#ifndef FRAMEVIEW2D_H
#define FRAMEVIEW2D_H

#include <QWidget>
#include <QGraphicsView>
#include <QGraphicsScene>

class FrameView2D : public QWidget
{
    Q_OBJECT
private:
    class TestGraphicsView : public QGraphicsView
    {
    public:
        TestGraphicsView(QWidget* parent = 0):
            QGraphicsView(parent){
            setDragMode(DragMode::ScrollHandDrag);
        }
        TestGraphicsView(QGraphicsScene* scene, QWidget* parent = 0):
            QGraphicsView(scene,parent){
            setDragMode(DragMode::ScrollHandDrag);
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
    TestGraphicsView *view;
    QGraphicsScene *scene;
};
#endif // FRAMEVIEW2D_H
