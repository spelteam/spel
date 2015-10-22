#ifndef UTILITY_H
#define UTILITY_H

// SPEL definitions
#include "predef.hpp"

//#include <functional>
#include <QObject>
#include <QString>
#include <QDomDocument>
#include <QColor>
#include <QPixmap>
#include <QImage>
#include <QBitmap>

#include <bodyJoint.hpp>
#include <bodyPart.hpp>
#include <bodyjointitem.h>
#include <bodypartitem.h>

namespace posegui {

  class Utility : public QObject
  {
    Q_OBJECT
  public:
    explicit Utility(QObject *parent = 0);
    ~Utility();

  signals:

    public slots :

  public:
    static QString fileToString(const QString& filename);

    //static void loadXmlPart( const QDomElement& element, const QString& partTagName,
    //          std::function<void( const QDomElement&, int )> loadAttrs);

    static BodyJoint* getJointById(const tree<BodyJoint>& bodyJoints, int id);
    static BodyPart* getBodyPartById(const tree<BodyPart>& bodyParts, int id);

    static BodyJointItem* getJointItemById(const QList<QGraphicsItem*>& items, int id);

    static void resizeImage(cv::Mat& image, int32_t& cols, int32_t& rows);

    static bool isJointItem(const QList<QGraphicsItem*>::iterator& it);
    static bool isSkeletonItem(const QList<QGraphicsItem*>::iterator& it);
    static QColor blendColors(const QColor& first, const QColor& second);

    static void buildBodyPartTree(std::vector<BodyPart> &bodyList,
      tree<BodyPart> &bodyParts);

    static QPixmap loadMask(QImage mask, int opacity);
  };

}

#endif // UTILITY_H
