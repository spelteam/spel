#include "framehandler.h"

#include <QDomElement>
#include <QDomDocument>
#include "frameattrstorage.h"
#include "projectattr.h"

namespace posegui {

FrameAttrStorage FrameHandler::read( const QDomElement &data ){
    FrameAttrStorage frame;
    const QDomNamedNodeMap& attrs = data.attributes();

    frame.id = attrs.namedItem(FrameAttrs::ID)
            .nodeValue().toInt();
    frame.imgPath = attrs.namedItem(FrameAttrs::IMG_PATH)
            .nodeValue();
    frame.maskPath = attrs.namedItem(FrameAttrs::MASK_PATH)
            .nodeValue();
    frame.camPath = attrs.namedItem(FrameAttrs::CAM_PATH)
            .nodeValue();
    frame.gpX = attrs.namedItem(FrameAttrs::GPX)
            .nodeValue().toFloat();
    frame.gpY = attrs.namedItem(FrameAttrs::GPY)
            .nodeValue().toFloat();
    QString isKeyframe = attrs.namedItem(FrameAttrs::IS_KEY)
            .nodeValue();
    frame.isKeyframe = ( isKeyframe == "true" || isKeyframe == "1" );

    return frame;
}

QDomElement FrameHandler::write(const FrameAttrStorage &model , QDomDocument &controller){
    QDomElement elem = controller
            .createElement(FrameAttrs::FRAME_TAG);
    elem.setAttribute(FrameAttrs::ID, model.id);
    elem.setAttribute(FrameAttrs::IMG_PATH, model.imgPath);
    elem.setAttribute(FrameAttrs::MASK_PATH, model.maskPath);
    elem.setAttribute(FrameAttrs::CAM_PATH, model.camPath);
    elem.setAttribute(FrameAttrs::GPX, model.gpX);
    elem.setAttribute(FrameAttrs::GPY, model.gpY);
    elem.setAttribute(FrameAttrs::IS_KEY, model.isKeyframe);
    return elem;
}

}

