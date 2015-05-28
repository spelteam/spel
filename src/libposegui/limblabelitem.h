#ifndef LIMBLABELITEM_H
#define LIMBLABELITEM_H

#include <QGraphicsItem>
#include <solvlet.hpp>

class LimbLabelItem : public QGraphicsItem
{
public:
    LimbLabelItem( const LimbLabel* label, QGraphicsItem * parent = 0 );
    ~LimbLabelItem();
public:
    enum { Type = UserType + 3 };
    int type() const { return Type; }

    QRectF boundingRect() const override;

    void paint( QPainter *painter, const QStyleOptionGraphicsItem *option,
                QWidget *widget = 0 ) override;
private:
    void updateToolTip();
private:
    const LimbLabel* label;
};

#endif // LIMBLABELITEM_H
