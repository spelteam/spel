#ifndef PROJECTHANDLER_H
#define PROJECTHANDLER_H

#include "modelhandler.h"

class QDomElement;
class QDomDocument;
struct ProjectAttrStorage;

namespace posegui {

class ProjectHandler : public ModelHandler<ProjectAttrStorage,QDomElement,QDomDocument>
{
public:
    ProjectAttrStorage read(const QDomElement &data) override;
    QDomElement write(const ProjectAttrStorage &model, QDomDocument& controller) override;
    virtual ~ProjectHandler() override{}
};

}

#endif // PROJECTHANDLER_H
