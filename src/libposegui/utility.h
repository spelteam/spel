#ifndef UTILITY_H
#define UTILITY_H

#include <QObject>
#include <QString>
#include <QDomDocument>

#include <bodyJoint.hpp>
#include <bodyPart.hpp>

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

    static BodyJoint& getJointById( tree<BodyJoint>& bodyJoints, int id );
    static BodyPart& getBodyPartById( tree<BodyPart>& bodyParts, int id );

    static void resizeImage( cv::Mat& image, int32_t& cols, int32_t& rows );
};

#endif // UTILITY_H
