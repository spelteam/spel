#include "bodypartsheaderhandler.h"

#include <QDomDocument>
#include <QDomElement>
#include <bodyPart.hpp>
#include "projectattr.h"
#include "utility.h"
#include "exceptions.h"

namespace posegui {

std::vector<BodyPart> BodyPartsHeaderHandler::read(const QDomElement &data, const tree<BodyJoint> &checkJoints){
    std::vector<BodyPart> bodyParts;
    while (!data.isNull()) {
        BodyPart bodyPart = partHandler.read(data);
        if( Utility::getJointById(checkJoints, bodyPart.getParentJoint()) == nullptr ||
            Utility::getJointById(checkJoints, bodyPart.getChildJoint()) == nullptr )
        {
            throw InvalidProjectStructure("Value of parent or child joint "
                                          "of body part doesn't exist");
        }
        bodyParts.push_back(bodyPart);
        //go to next Part
        const_cast<QDomElement&>(data) = data.nextSiblingElement();
    }
    return bodyParts;
}

QDomElement BodyPartsHeaderHandler::write(const tree<BodyPart> &model, QDomDocument& controller){
    QDomElement elem = controller
            .createElement(BodyPartHeaderAttrs::PARTS_TAG);
    tree<BodyPart>::iterator it = model.begin();
    while( it != model.end() ){
        QDomElement childElem = partHandler.write(*it,controller);
        elem.appendChild(childElem);
        ++it;
    }
    return elem;
}

}

