#ifndef BODYPARTHANDLER_H
#define BODYPARTHANDLER_H

#include "modelhandler.h"

class QDomElement;
class QDomDocument;
class BodyPart;

namespace posegui {

class BodyPartHandler : public ModelHandler<BodyPart,QDomElement,QDomDocument>
{
public:
    BodyPart read(const QDomElement &data) override;
    QDomElement write(const BodyPart &model, QDomDocument &controller) override;
    virtual ~BodyPartHandler() override{}
};

}

#endif // BODYPARTHANDLER_H
