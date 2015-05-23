#ifndef BODYPARTSHANDLER_H
#define BODYPARTSHANDLER_H

#include "modelhandler.h"
#include "bodyparthandler.h"
#include <tree.hh>

class QDomElement;
class QDomDocument;
class BodyPart;
class BodyJoint;

namespace posegui {

class BodyPartsHandler : public ModelHandler<tree<BodyPart>,QDomElement,QDomDocument,void>
{
public:
    void read( const QDomElement &data, tree<BodyPart> &bodyParts );
    QDomElement write(const tree<BodyPart> &model, QDomDocument &controller );
    virtual ~BodyPartsHandler() override{}
private:
    BodyPartHandler partHandler;
};

}

#endif // BODYPARTSHANDLER_H
