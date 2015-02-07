#include "utility.h"

#include <QFile>
#include <QtGlobal>

Utility::Utility(QObject *parent) : QObject(parent)
{

}

Utility::~Utility()
{}

QString Utility::fileToString(const QString &filename){
    QFile file(filename);
    file.open( QFile::ReadOnly );
    QString result(file.readAll());
    file.close();

    return result;
}

void Utility::loadXmlPart(const QDomElement &element, const QString& partTagName,
                std::function<void(const QDomElement&, int)> loadAttrs)
{
    const QDomNodeList& partNodes = element
            .elementsByTagName(partTagName).at(0)
            .childNodes();
    for(int i = 0; i < partNodes.size(); i++ ){
        const QDomElement& childNode = partNodes.at(i).toElement();

        loadAttrs( childNode, i );
    }
}

BodyJoint& Utility::getJointById(tree<BodyJoint> &bodyJoints, int id){
    tree<BodyJoint>::iterator it;
    it = bodyJoints.begin();
    while( it != bodyJoints.end() ){
        if( it->getLimbID() == id ) return *it;
        it++;
    }
    Q_ASSERT(false);
}

BodyPart& Utility::getBodyPartById(tree<BodyPart> &bodyParts, int id){
    tree<BodyPart>::iterator it;
    it = bodyParts.begin();
    while( it != bodyParts.end() ){
        if( it->getPartID() == id ) return *it;
        it++;
    }
    Q_ASSERT(false);
}

void Utility::resizeImage(Mat &image, int32_t &cols, int32_t &rows){
    if( cols <= 0 || rows <= 0 ){
        cols = image.cols;
        rows = image.rows;
    }
    if( cols != image.cols || rows != image.rows ){
        cv::resize(image, image, cv::Size(cols, rows) );
    }
}
