#ifndef BODYPARTITEM_H
#define BODYPARTITEM_H

#include <QGraphicsItem>
#include <frame.hpp>

class BodyJointItem;
class BodyPart;
class LimbLabel;

class BodyPartItem : public QGraphicsItem
{
private:
    struct Palette{
        static QColor occluded;
        static QColor notOccluded;
        static QColor interpolation;
        static QColor lockframe;
        static QColor keyframe;
        static QColor selected;
    };
public:
    BodyPartItem(BodyPart *bodyPart, BodyJointItem *parentJoint,
                  BodyJointItem *childJoint);
    ~BodyPartItem();
signals:
    void updatePart(BodyPartItem *partItem);
public:
    void adjust();

    enum { Type = UserType + 2 };
    int type() const { return Type; }

    QRectF boundingRect() const override;
    void paint( QPainter *painter, const QStyleOptionGraphicsItem *option,
                QWidget *widget = 0 ) override;
protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *event) override;

    void hoverEnterEvent(QGraphicsSceneHoverEvent *event) override;
    void hoverLeaveEvent(QGraphicsSceneHoverEvent *event) override;
public:
    static FRAMETYPE Frametype;
private:
    void updateToolTip();
private:
    BodyJointItem* source;
    BodyJointItem* dest;
    BodyPart* bodyPart;
};

#endif // BODYPARTITEM_H
