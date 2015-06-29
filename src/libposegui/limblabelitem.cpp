#include "limblabelitem.h"

#include <QPainter>
#include <QStyleOptionGraphicsItem>
#include "utility.h"
#include <limbLabel.hpp>

//PUBLIC

FRAMETYPE LimbLabelItem::Frametype = LOCKFRAME;

LimbLabelItem::LimbLabelItem( LimbLabel label, QGraphicsItem *parent)
    : QGraphicsItem( parent ),
      label(label)
{
    setAcceptedMouseButtons(Qt::NoButton);
    //set position
    Point2f center = label.getCenter();
    setPos(center.x, center.y);
    setZValue(1.5);
    setAcceptHoverEvents(true);
}

LimbLabelItem::~LimbLabelItem()
{
}
//TODO[!] Rect in local space
#include <QDebug>
QRectF LimbLabelItem::boundingRect() const{
    qreal offset = 0.3;
    std::vector<Point2f> polygon = label.getPolygon();
    // [0] - top & right
    // [1] - top & left
    // [2] - bottom & left
    // [3] - bottom & right
    qreal width = polygon[0].x - polygon[1].x;
    qreal height = polygon[2].y - polygon[0].y;

    qDebug() << "LimbLabelItem center:" << endl;
    qDebug() << "center: " << label.getCenter().x << " " << label.getCenter().y << endl;
    /*qDebug() << "LimbLabelItem bounding rect:" << endl;
    qDebug() << "top:" << top << endl;
    qDebug() << "left:" << left << endl;
    qDebug() << "width:" << width << endl;
    qDebug() << "height:" << height << endl;*/

    return QRectF(-width/2, -height/2, width, height)
            .normalized()
            .adjusted(-offset, -offset, offset, offset);
}

void LimbLabelItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
                          QWidget *widget)
{
    //set color
    QColor color;
    switch(Frametype){
    case INTERPOLATIONFRAME:
        color = Palette::interpolation;
        break;
    case KEYFRAME:
        color = Palette::keyframe;
        break;
    case LOCKFRAME:
        color = Palette::lockframe;
        break;
    }
    //set width
    qreal width = 0.0;
    if( option->state & QStyle::State_MouseOver ){
        color = posegui::Utility::blendColors(color,Palette::selected);
        width = 0.7;
    }
    //set pen
    painter->setPen(QPen(color,width));
    //draw
    std::vector<Point2f> polygon = label.getPolygon();
    painter->drawLine(polygon[0].x,polygon[0].y,polygon[1].x,polygon[1].y);
    painter->setPen(Qt::red);
    painter->drawLine(polygon[1].x,polygon[1].y,polygon[2].x,polygon[2].y);
    painter->setPen(Qt::black);
    painter->drawLine(polygon[2].x,polygon[2].y,polygon[3].x,polygon[3].y);
    painter->setPen(Qt::cyan);
    painter->drawLine(polygon[3].x,polygon[3].y,polygon[0].x,polygon[0].y);
    painter->drawRect(boundingRect());
}

//PRIVATE

QColor LimbLabelItem::Palette::interpolation = Qt::blue;
QColor LimbLabelItem::Palette::lockframe = Qt::green;
QColor LimbLabelItem::Palette::keyframe = Qt::red;
QColor LimbLabelItem::Palette::selected = Qt::white;


void LimbLabelItem::updateToolTip(){

}

