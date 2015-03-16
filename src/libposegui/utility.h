#ifndef UTILITY_H
#define UTILITY_H

#include <QObject>
#include <QString>
#include <QDomDocument>
#include <QColor>

#include <bodyJoint.hpp>
#include <bodyPart.hpp>
#include <bodyjointitem.h>
#include <bodypartitem.h>

class Utility : public QObject
{
    Q_OBJECT
public:
    explicit Utility(QObject *parent = 0);
    ~Utility();

signals:

public slots:

public:
    static QString fileToString( const QString& filename );

    static void loadXmlPart( const QDomElement& element, const QString& partTagName,
              std::function<void( const QDomElement&, int )> loadAttrs);

    static BodyJoint* getJointById( const tree<BodyJoint>& bodyJoints, int id );
    static BodyPart* getBodyPartById( const tree<BodyPart>& bodyParts, int id );

    static BodyJointItem* getJointItemById( const QList<QGraphicsItem*>& items, int id );

    static void resizeImage( cv::Mat& image, int32_t& cols, int32_t& rows );

    static bool isJointItem( QList<QGraphicsItem*>::iterator& it );
    static bool isSkeletonItem( QList<QGraphicsItem*>::iterator it );
    static QColor blendColors( const QColor& first, const QColor& second );
};

#endif // UTILITY_H
