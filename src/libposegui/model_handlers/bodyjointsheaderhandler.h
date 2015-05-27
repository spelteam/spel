#ifndef BODYJOINTSHEADERHANDLER_H
#define BODYJOINTSHEADERHANDLER_H

#include "modelhandler.h"
#include "bodyjointheaderhandler.h"
#include <tree.hh>

class QDomElement;
class QDomDocument;
class BodyJoint;

namespace posegui {

class BodyJointsHeaderHandler : public ModelHandler<tree<BodyJoint>,QDomElement,QDomDocument>
{
public:
    tree<BodyJoint> read( const QDomElement &data) override;
    QDomElement write(const tree<BodyJoint> &model, QDomDocument& controller ) override;
    virtual ~BodyJointsHeaderHandler() override{}
private:
    BodyJointHeaderHandler jointHandler;
};

}

#endif // BODYJOINTSHEADERHANDLER_H
