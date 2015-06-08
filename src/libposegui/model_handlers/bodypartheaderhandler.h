#ifndef BODYPARTHEADERHANDLER_H
#define BODYPARTHEADERHANDLER_H

#include "modelhandler.h"

class QDomElement;
class QDomDocument;
class BodyPart;

namespace posegui {

class BodyPartHeaderHandler : public ModelHandler<BodyPart,QDomElement,QDomDocument>
{
public:
    BodyPart read(const QDomElement &data) override;
    QDomElement write(const BodyPart &model, QDomDocument &controller) override;
    virtual ~BodyPartHeaderHandler() override{}
};

}
#endif // BODYPARTHEADERHANDLER_H
