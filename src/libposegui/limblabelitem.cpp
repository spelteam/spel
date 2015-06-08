#include "limblabelitem.h"

LimbLabelItem::LimbLabelItem(const LimbLabel *label, QGraphicsItem *parent)
    : QGraphicsItem( parent ),
      label(label)
{
    setAcceptedMouseButtons(Qt::NoButton);
    setZValue(1.5);
}

QRectF LimbLabelItem::boundingRect()const{
    return QRectF(10,10,10,10);
}

void LimbLabelItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
                          QWidget *widget)
{

}

LimbLabelItem::~LimbLabelItem()
{
}

