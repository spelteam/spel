#include "bodyjointshandler.h"

#include <QDomElement>
#include <QDomDocument>
#include <bodyJoint.hpp>
#include "projectattr.h"
#include "utility.h"
#include "exceptions.h"

namespace posegui {

void BodyJointsHandler::read(const QDomElement &data, tree<BodyJoint> &joints,
                             float colsFactor, float rowsFactor)
{
    while (!data.isNull()) {
        BodyJoint joint = jointHandler.read(data);
        BodyJoint* origJoint = Utility::getJointById(joints,joint.getLimbID());
        if( origJoint == nullptr ){
            throw InvalidProjectStructure("Can't find joint with id: " +
                                          joint.getLimbID());
        }
        Point2f imgLocation = joint.getImageLocation();
        imgLocation.x *= colsFactor;
        imgLocation.y *= rowsFactor;
        origJoint->setImageLocation(imgLocation);
        origJoint->setDepthSign(joint.getDepthSign());
        //go to next joint
        const_cast<QDomElement&>(data) = data.nextSiblingElement();
    }
}

QDomElement BodyJointsHandler::write(const tree<BodyJoint> &model, QDomDocument &controller){
    QDomElement elem = controller
            .createElement(BodyJointAttrs::JOINTS_TAG);
    tree<BodyJoint>::iterator it = model.begin();
    while( it != model.end() ){
        QDomElement childElem = jointHandler.write(*it,controller);
        elem.appendChild(childElem);
        ++it;
    }
    return elem;
}

}

