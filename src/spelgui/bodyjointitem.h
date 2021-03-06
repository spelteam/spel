#ifndef BODYJOINTITEM_H
#define BODYJOINTITEM_H

#include <QGraphicsItem>
#include <frame.hpp>
#include <bodyJoint.hpp>

//TODO[!]: Add state machine 

using namespace SPEL;

class BodyPartItem;

class BodyJointItem : public QGraphicsItem
{
private:
  struct Palette{
    static QColor inDepth; //depthSign true
    static QColor outDepth;//depthSign false
    static QColor selected;
  };
public:
  BodyJointItem(BodyJoint *joint, QGraphicsItem * parent = 0);
  ~BodyJointItem();
signals:
  void updateJoint(BodyJointItem *jointItem);
public:
  void addBodyPart(BodyPartItem *bodyPart);
  int getId() const;

  enum { Type = UserType + 1 };
  int type() const { return Type; }

  QRectF boundingRect() const override;

  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
    QWidget *widget = 0) override;
protected:
  QVariant itemChange(GraphicsItemChange change, const QVariant &value) override;

  void mousePressEvent(QGraphicsSceneMouseEvent *event) override;
  void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) override;

  void hoverEnterEvent(QGraphicsSceneHoverEvent *event) override;
  void hoverLeaveEvent(QGraphicsSceneHoverEvent *event) override;
private:
  void updateToolTip();
public:
  static FRAMETYPE Frametype;
private:
  QList<BodyPartItem*> partList;
  qreal radius = 1.0;
  BodyJoint *joint;
};

#endif // BODYJOINTITEM_H
