// SPEL definitions
#include "predef.hpp"

#include "bodypartshandler.h"

#include <QDomDocument>
#include <QDomElement>
#include "projectattr.h"
#include "utility.h"
#include "exceptions.h"

namespace posegui {

  void BodyPartsHandler::read(const QDomElement &data, tree<BodyPart> &bodyParts){
    while (!data.isNull()) {
      BodyPart bodyPart = partHandler.read(data);
      BodyPart* origBodyPart = Utility::getBodyPartById(bodyParts, bodyPart.getPartID());
      if (origBodyPart == nullptr){
        throw InvalidProjectStructure("Can't find body part with id: " +
          bodyPart.getPartID());
      }
      origBodyPart->setIsOccluded(bodyPart.getIsOccluded());
      //go to next Part
      const_cast<QDomElement&>(data) = data.nextSiblingElement();
    }
  }

  QDomElement BodyPartsHandler::write(const tree<BodyPart> &model, QDomDocument& controller){
    QDomElement elem = controller
      .createElement(BodyPartAttrs::PARTS_TAG);
    tree<BodyPart>::iterator it = model.begin();
    while (it != model.end()){
      QDomElement childElem = partHandler.write(*it, controller);
      elem.appendChild(childElem);
      ++it;
    }
    return elem;
  }

}

