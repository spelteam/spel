#include "bodyjointheaderhandler.h"

#include <QDomElement>
#include <QDomDocument>
#include "projectattr.h"

namespace posegui {

  BodyJoint BodyJointHeaderHandler::read(const QDomElement &data){
    BodyJoint bodyJoint;
    const QDomNamedNodeMap& attrs = data.attributes();

    int id = attrs.namedItem(BodyJointHeaderAttrs::ID)
      .nodeValue()
      .toInt();
    QString name = attrs.namedItem(BodyJointHeaderAttrs::NAME)
      .nodeValue();

    bodyJoint.setLimbID(id);
    bodyJoint.setJointName(name.toStdString());
    return bodyJoint;
  }

  QDomElement BodyJointHeaderHandler::write(const BodyJoint &model, QDomDocument &controller){
    QDomElement elem = controller
      .createElement(BodyJointHeaderAttrs::JOINT_TAG);
    elem.setAttribute(BodyJointHeaderAttrs::ID, model.getLimbID());
    elem.setAttribute(BodyJointHeaderAttrs::NAME, model.getJointName().c_str());
    //qDebug() << "BodyJoint xml:" << elem.nodeValue() << endl;
    return elem;
  }

}

