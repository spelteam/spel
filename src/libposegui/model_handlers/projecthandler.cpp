#include "projecthandler.h"

#include <QDomElement>
#include <QDomDocument>
#include "projectattrstorage.h"
#include "projectattr.h"

namespace posegui {

ProjectAttrStorage ProjectHandler::read( const QDomElement &data ){
    ProjectAttrStorage project;
    const QDomNamedNodeMap& attrs = data.attributes();

    project.name = attrs.namedItem(ProjectAttrs::NAME)
            .nodeValue();
    project.imgFolderPath = attrs.namedItem(ProjectAttrs::IMG_FOLDER)
            .nodeValue();
    project.maskFolderPath = attrs.namedItem(ProjectAttrs::MASK_FOLDER)
            .nodeValue();
    project.camFolderPath = attrs.namedItem(ProjectAttrs::CAM_FOLDER)
            .nodeValue();
    QString allowScaling = attrs.namedItem(ProjectAttrs::ALLOW_SCALLING)
            .nodeValue();
    project.allowScaling = ( allowScaling=="true" || allowScaling=="1" );
    project.simMatPath = attrs.namedItem(ProjectAttrs::SIM_MAT_PATH)
            .nodeValue();
    project.exportPath = attrs.namedItem(ProjectAttrs::EXPORT_PATH)
            .nodeValue();

    return project;
}

QDomElement ProjectHandler::write(const ProjectAttrStorage &model , QDomDocument &controller){
    QDomElement elem = controller
            .createElement(ProjectAttrs::PROJECT_TAG);
    elem.setAttribute(ProjectAttrs::NAME, model.name);
    elem.setAttribute(ProjectAttrs::IMG_FOLDER, model.imgFolderPath);
    elem.setAttribute(ProjectAttrs::MASK_FOLDER, model.maskFolderPath);
    elem.setAttribute(ProjectAttrs::CAM_FOLDER, model.camFolderPath);
    elem.setAttribute(ProjectAttrs::ALLOW_SCALLING, model.allowScaling);
    elem.setAttribute(ProjectAttrs::SIM_MAT_PATH, model.simMatPath);
    elem.setAttribute(ProjectAttrs::EXPORT_PATH, model.exportPath);
    return elem;
}

}

