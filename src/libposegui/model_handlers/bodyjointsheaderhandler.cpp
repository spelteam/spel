#include "bodyjointsheaderhandler.h"

#include <QDomElement>
#include <QDomDocument>
#include <bodyJoint.hpp>
#include "projectattr.h"

namespace posegui {

tree<BodyJoint> BodyJointsHeaderHandler::read(const QDomElement &data ){
    tree<BodyJoint> joints;
    tree<BodyJoint>::iterator jointsTop = joints.begin();
    while (!data.isNull()) {
        BodyJoint joint = jointHandler.read(data);
        joints.insert(jointsTop, joint);
        //go to next joint
        const_cast<QDomElement&>(data) = data.nextSiblingElement();
    }
    return joints;
}

QDomElement BodyJointsHeaderHandler::write(const tree<BodyJoint> &model , QDomDocument &controller){
    QDomElement elem = controller
            .createElement(BodyJointHeaderAttrs::JOINTS_TAG);
    tree<BodyJoint>::iterator it = model.begin();
    while( it != model.end() ){
        QDomElement childElem = jointHandler.write(*it,controller);
        elem.appendChild(childElem);
        ++it;
    }
    return elem;
}

}
