#ifndef BODYJOINTITEM_H
#define BODYJOINTITEM_H

#include <QGraphicsItem>

class BodyJointItem : public QGraphicsItem
{
public:
    BodyJointItem();
    ~BodyJointItem();
public:
    QRectF boundingRect() const override;
    void paint( QPainter *painter, const QStyleOptionGraphicsItem *option,
                QWidget *widget = 0 ) override;
private:
    qreal radius = 1.0;
};

#endif // BODYJOINTITEM_H
