#ifndef LIMBLABELITEM_H
#define LIMBLABELITEM_H

#include <QGraphicsItem>
#include <frame.hpp>
#include <limbLabel.hpp>

using namespace SPEL;

class LimbLabelItem : public QGraphicsItem
{
private:
  struct Palette{
    static QColor interpolation;
    static QColor lockframe;
    static QColor keyframe;
    static QColor selected;
  };
public:
  LimbLabelItem(LimbLabel label, QGraphicsItem * parent = 0);
  ~LimbLabelItem();
public:
  enum { Type = UserType + 3 };
  int type() const { return Type; }

  QRectF boundingRect() const override;

  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
    QWidget *widget = 0) override;
private:
  void updateToolTip();
public:
  static FRAMETYPE Frametype;
private:
  const LimbLabel label;
};

#endif // LIMBLABELITEM_H
