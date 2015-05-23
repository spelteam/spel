    #ifndef BODYJOINTHANDLER_H
#define BODYJOINTHANDLER_H

#include "modelhandler.h"

class QDomElement;
class QDomDocument;
class BodyJoint;

namespace posegui {

class BodyJointHandler : public ModelHandler<BodyJoint,QDomElement,QDomDocument>
{
public:
    BodyJoint read(const QDomElement &data) override;
    QDomElement write(const BodyJoint &model, QDomDocument &controller) override;
    virtual ~BodyJointHandler() override{}
};

}

#endif // BODYJOINTHANDLER_H
