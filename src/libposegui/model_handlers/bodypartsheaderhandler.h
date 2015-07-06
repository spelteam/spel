#ifndef BODYPARTSHEADERHANDLER_H
#define BODYPARTSHEADERHANDLER_H

#include "modelhandler.h"
#include "bodypartheaderhandler.h"
#include <tree.hh>
#include <vector>

class QDomElement;
class QDomDocument;
class BodyPart;
class BodyJoint;

namespace posegui {

class BodyPartsHeaderHandler : public ModelHandler<tree<BodyPart>,QDomElement,tree<BodyJoint>,QDomDocument>
{
public:
    std::vector<BodyPart> read( const QDomElement &data, const tree<BodyJoint> &checkJoints );
    QDomElement write(const tree<BodyPart> &model, QDomDocument& controller );
    virtual ~BodyPartsHeaderHandler() override{}
private:
    BodyPartHeaderHandler partHandler;
};

}

#endif // BODYPARTSHEADERHANDLER_H
