#ifndef BODYJOINTITEM_H
#define BODYJOINTITEM_H

#include <QGraphicsItem>

class BodyPartItem;

class BodyJointItem : public QGraphicsItem
{
public:
    BodyJointItem(  QGraphicsItem * parent = 0 );
    ~BodyJointItem();
public:
    void addBodyPart( BodyPartItem *bodyPart );
    void setId( int id );
    int getId() const;

    enum { Type = UserType + 1 };
    int type() const { return Type; }

    QRectF boundingRect() const override;

    void paint( QPainter *painter, const QStyleOptionGraphicsItem *option,
                QWidget *widget = 0 ) override;
protected:
    QVariant itemChange(GraphicsItemChange change, const QVariant &value) override;

    void mousePressEvent(QGraphicsSceneMouseEvent *event) override;
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) override;
private:
    QList<BodyPartItem*> partList;
    qreal radius = 1.0;
    int id;
};

#endif // BODYJOINTITEM_H
