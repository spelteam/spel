#ifndef FRAMEHANDLER_H
#define FRAMEHANDLER_H

#include "modelhandler.h"
#include <memory>

class QDomElement;
class QDomDocument;
struct FrameAttrStorage;

namespace posegui {

class FrameHandler : public ModelHandler<FrameAttrStorage,QDomElement,QDomDocument>
{
public:
    FrameAttrStorage read(const QDomElement &data) override;
    QDomElement write(const FrameAttrStorage &model, QDomDocument& controller) override;
    virtual ~FrameHandler() override{}
};

}

#endif // FRAMEHANDLER_H
