#include "bodypartheaderhandler.h"

#include <QDomElement>
#include <QDomDocument>
#include <bodyPart.hpp>
#include "projectattr.h"

namespace posegui {

BodyPart BodyPartHeaderHandler::read( const QDomElement &data ){
    BodyPart bodyPart;
    const QDomNamedNodeMap& attrs = data.attributes();

    int id = attrs.namedItem(BodyPartHeaderAttrs::ID)
            .nodeValue().toInt();
    QString name = attrs.namedItem(BodyPartHeaderAttrs::NAME)
            .nodeValue();
    int childJointId = attrs.namedItem(BodyPartHeaderAttrs::CHDJOINT)
                .nodeValue().toInt();
    int parentJointId = attrs.namedItem(BodyPartHeaderAttrs::PRTJOINT)
            .nodeValue().toInt();
    float lwRatio = attrs.namedItem(BodyPartHeaderAttrs::LWRATIO)
            .nodeValue().toFloat();
    float expectedDistance = attrs.namedItem(BodyPartHeaderAttrs::EXPECT_DIST)
            .nodeValue().toFloat();
    float relativeLength = attrs.namedItem(BodyPartHeaderAttrs::REL_LENGTH)
            .nodeValue().toFloat();

    bodyPart.setPartID(id);
    bodyPart.setPartName(name.toStdString());
    bodyPart.setParentJoint(parentJointId);
    bodyPart.setChildJoint(childJointId);
    bodyPart.setLWRatio(lwRatio);
    bodyPart.setExpectedDistance(expectedDistance);
    bodyPart.setRelativeLength(relativeLength);

    return bodyPart;
}

QDomElement BodyPartHeaderHandler::write(const BodyPart &model , QDomDocument &controller){
    QDomElement elem = controller
            .createElement(BodyPartHeaderAttrs::PART_TAG);
    elem.setTagName(BodyPartHeaderAttrs::PART_TAG);
    elem.setAttribute(BodyPartHeaderAttrs::ID,model.getPartID());
    elem.setAttribute(BodyPartHeaderAttrs::NAME,model.getPartName().c_str());
    elem.setAttribute(BodyPartHeaderAttrs::CHDJOINT,model.getChildJoint());
    elem.setAttribute(BodyPartHeaderAttrs::PRTJOINT,model.getParentJoint());
    elem.setAttribute(BodyPartHeaderAttrs::LWRATIO,model.getLWRatio());
    elem.setAttribute(BodyPartHeaderAttrs::EXPECT_DIST,model.getExpectedDistance());
    elem.setAttribute(BodyPartHeaderAttrs::REL_LENGTH,model.getRelativeLength());
    return elem;
}

}

