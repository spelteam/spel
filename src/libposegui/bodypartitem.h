#ifndef BODYPARTITEM_H
#define BODYPARTITEM_H

#include <QGraphicsItem>

class BodyJointItem;

class BodyPartItem : public QGraphicsItem
{
public:
    BodyPartItem( BodyJointItem *parentJoint,
                  BodyJointItem *childJoint );
    ~BodyPartItem();
public:
    void adjust();
    void setId( int id );

    enum { Type = UserType + 2 };
    int type() const { return Type; }

    QRectF boundingRect() const override;
    void paint( QPainter *painter, const QStyleOptionGraphicsItem *option,
                QWidget *widget = 0 ) override;
protected:
    void hoverEnterEvent(QGraphicsSceneHoverEvent *event) override;
    void hoverLeaveEvent(QGraphicsSceneHoverEvent *event) override;
private:
    BodyJointItem* source;
    BodyJointItem* dest;
    int id;
};

#endif // BODYPARTITEM_H
