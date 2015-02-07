#include "project.h"

#include <QFile>
#include <QXmlSchema>
#include <QXmlSchemaValidator>
#include "xmlmessagehandler.h"

#include <keyframe.hpp>
#include <interpolation.hpp>
#include <lockframe.hpp>

#include "projectattr.h"
#include "utility.h"

using namespace project_attributes;

//STATIC
QString FilenamePath::imgFolderPath = "";
QString FilenamePath::maskFolderPath = "";

//PUBLIC

Project& Project::getInstance(){
    static Project instance;
    return instance;
}

std::vector<Frame*> Project::getFrames() const{
    std::vector<Frame*> framesPtr;

    for( const FramePtr& currFrame : frames ){
        framesPtr.push_back(currFrame.get());
    }

    return framesPtr;
}

#include <QDebug>
//TODO: [?]Go to background thread
//TODO: [L]Create status bar of loading
Project::ErrorCode Project::openProjectEvent(const QString &filename, QString *errMessage){
    QRegExp rx("^/.*/");
    rx.setMinimal(false);
    projectFolder = filename.split(
        filename.split(rx,QString::SkipEmptyParts).first(),
        QString::SkipEmptyParts
    ).first();
    qDebug() << projectFolder << endl;
    //TODO: [!]Refactor method
    //read content from file
    QDomDocument document;
    QFile file(filename);
    if( file.open(QFile::ReadOnly) ){
        document.setContent(&file);
    } else{
        if(errMessage) *errMessage = file.errorString();
        return ErrorCode::FILE_NOT_OPEN;
    }
    file.close();

    //validate document
    //set schema data
    file.setFileName(":/root/resources/xml/project.xsd");
    QXmlSchema schema;
    XmlMessageHandler messagesHandler;
    schema.setMessageHandler(&messagesHandler);
    if( file.open(QFile::ReadOnly) ){
        schema.load(&file);
    } else{
        if(errMessage) *errMessage = file.errorString();
        return ErrorCode::FILE_NOT_OPEN;
    }
    file.close();
    //check for errors
    bool errorOccured = false;
    if(!schema.isValid()){
        errorOccured = true;
    } else{
        QXmlSchemaValidator validator(schema);
        if( !validator.validate(document.toByteArray()) ){
                errorOccured = true;
        }
    }
    if(errorOccured){
        if(errMessage) *errMessage = messagesHandler.errorMessage();
        return ErrorCode::INVALID_PROJECT_STRUCTURE;
    }

    //load paths and frames
    const QDomElement& projectElem = document.documentElement();
    const QDomNamedNodeMap& projectAttrs = projectElem.attributes();
    //set folder path
    FilenamePath::imgFolderPath = projectAttrs.namedItem(PROJECT_IMG_FOLD_ATTR).nodeValue();
    FilenamePath::maskFolderPath = projectAttrs.namedItem(PROJECT_MASK_FOLD_ATTR).nodeValue();
    qDebug() << FilenamePath::imgFolderPath << endl;
    qDebug() << FilenamePath::maskFolderPath << endl;
    //set project options
    //TODO: [L]Setup project options
    //load skeleton
    loadSkeleton(document);
    loadFrames(document);

    return ErrorCode::SUCCESS;
}


//PRIVATE

Project::Project(QObject *parent)
    :QObject(parent),
      frames(),
      paths(),
      skeleton( new Skeleton() )
{
    //connect
    QObject::connect(this, &Project::open, this, &Project::openProjectEvent);
}

void Project::loadSkeleton( const QDomDocument &document){
    //create bodyJoints tree
    tree<BodyJoint> bodyJoints;
    tree<BodyJoint>::iterator bodyJointsTop;
    bodyJointsTop = bodyJoints.begin();
    Utility::loadXmlPart( document.documentElement(), BODY_JOINTS_HEADER_TAG,
    [&bodyJoints,&bodyJointsTop]( const QDomElement& node, int ){
        const QDomNamedNodeMap& attrs = node.attributes();
        BodyJoint joint;

        int id = attrs.namedItem(BODY_JOINT_HEADER_ID_ATTR)
                .nodeValue()
                .toInt();
        QString name = attrs.namedItem(BODY_JOINT_HEADER_NAME_ATTR)
                .nodeValue();

        joint.setLimbID(id);
        joint.setJointName(name.toStdString());

        bodyJoints.insert( bodyJointsTop, joint);
    });
    //create bodyParts tree
    tree<BodyPart> bodyParts;
    tree<BodyPart>::iterator bodyPartsTop;
    bodyPartsTop = bodyParts.begin();
    Utility::loadXmlPart( document.documentElement(), BODY_PARTS_HEADER_TAG,
    [&bodyParts,&bodyPartsTop]( const QDomElement node, int ){
        const QDomNamedNodeMap& attrs = node.attributes();
        BodyPart bodyPart;

        int id = attrs.namedItem(BODY_PART_HEADER_ID_ATTR)
                .nodeValue().toInt();
        QString name = attrs.namedItem(BODY_PART_HEADER_NAME_ATTR)
                .nodeValue();
        int childJointId = attrs.namedItem(BODY_PART_HEADER_CHDJOINT_ATTR)
                .nodeValue().toInt();
        int parentJointId = attrs.namedItem(BODY_PART_HEADER_PRTJOINT_ATTR)
                .nodeValue().toInt();
        float lwRatio = attrs.namedItem(BODY_PART_HEADER_LWRATIO_ATTR)
                .nodeValue().toFloat();
        float expectedDistance = attrs.namedItem(BODY_PART_HEADER_EXPECTDIST_ATTR)
                .nodeValue().toFloat();
        //TODO: [?]check existance of joints in bodyJoints tree
        bodyPart.setPartID(id);
        bodyPart.setPartName(name.toStdString());
        bodyPart.setParentJoint(parentJointId);
        bodyPart.setChildJoint(childJointId);
        bodyPart.setLWRatio(lwRatio);
        bodyPart.setSpaceLength(expectedDistance);

        bodyParts.insert(bodyPartsTop, bodyPart);
    });

    skeleton->setJointTree(bodyJoints);
    skeleton->setPartTree(bodyParts);
    skeleton->setScale(100.f);
}

void Project::loadFrames( const QDomDocument &document ){
    int32_t firstFrameCol = -1;
    int32_t firstFrameRow = -1;
    Utility::loadXmlPart( document.documentElement(), FRAMES_TAG,
    [this,&firstFrameCol,&firstFrameRow]( const QDomElement& node, int num ){
        const QDomNamedNodeMap& attrs = node.attributes();

        int id = attrs.namedItem(FRAME_ID_ATTR).nodeValue().toInt();

        QString imgPath = attrs.namedItem(FRAME_IMG_PATH_ATTR).nodeValue();
        QString maskPath = attrs.namedItem(FRAME_MASK_PATH_ATTR).nodeValue();
        QString camPath = attrs.namedItem(FRAME_CAM_PATH_ATTR).nodeValue();
        paths[num] = { imgPath, maskPath, camPath };
        cv::Mat image = cv::imread(
            (projectFolder+FilenamePath::imgFolderPath+imgPath).toStdString(),
            CV_LOAD_IMAGE_COLOR
        );
        Q_ASSERT( image.data );
        int32_t imageOrigCols = image.cols;
        int32_t imageOrigRows = image.rows;
        Utility::resizeImage(image, firstFrameCol, firstFrameRow);
        cv::Mat mask = cv::imread(
            (projectFolder+FilenamePath::maskFolderPath+maskPath).toStdString(),
            CV_LOAD_IMAGE_GRAYSCALE
        );
        Q_ASSERT( mask.data );
        Utility::resizeImage(mask, firstFrameCol, firstFrameRow);
        qDebug() << "firstFrameCol: " << firstFrameCol << " "
                 << "firstFrameRow: " << firstFrameRow << endl;

        Point2f gp;
        gp.x = attrs.namedItem(FRAME_GPX_ATTR).nodeValue().toFloat();
        gp.y = attrs.namedItem(FRAME_GPY_ATTR).nodeValue().toFloat();

        QString isKeyframe = attrs.namedItem(FRAME_IS_KEY_ATTR)
                .nodeValue();

        FramePtr newFrame;
        //Test
        qDebug() << "Frame id: " << id << endl;
        //
        if( isKeyframe == "true" || isKeyframe == "1" ){
            newFrame = FramePtr( new Keyframe() );
            tree<BodyJoint> bodyJoints = skeleton->getJointTree();
            tree<BodyPart> bodyParts = skeleton->getPartTree();
            float colsFactor = static_cast<float>(image.cols) / imageOrigCols;
            float rowsFactor = static_cast<float>(image.rows) / imageOrigRows;
            qDebug() << "colsFactor: " <<  colsFactor << " "
                     << "rowsFactor: " << rowsFactor << endl;
            loadKeyframeJoints(node, bodyJoints, colsFactor, rowsFactor);
            loadKeyframeBodyParts(node, bodyParts);

            Skeleton currSkeleton;
            currSkeleton.setJointTree(bodyJoints);
            currSkeleton.setPartTree(bodyParts);
            currSkeleton.setScale(100.0);

            newFrame->setSkeleton(currSkeleton);
        } else{
            newFrame = FramePtr( new Interpolation() );

            newFrame->setSkeleton( *skeleton.get() );
        }

        newFrame->setID(id);
        newFrame->setGroundPoint(gp);
        newFrame->setImage(image);
        newFrame->setMask(mask);

        frames.push_back(std::move(newFrame));
    });
}

void Project::loadKeyframeJoints(const QDomElement &node, tree<BodyJoint> &bodyJoints,
                                 float colsFactor, float rowsFactor )
{
    Utility::loadXmlPart( node, BODY_JOINTS_TAG,
    [&bodyJoints,&colsFactor,&rowsFactor]( const QDomElement& currNode, int ){
        const QDomNamedNodeMap& attrs = currNode.attributes();

        QString depthSign = attrs.namedItem(BODY_JOINT_DEPTH_ATTR)
                .nodeValue();
        Point2f imgLocation;
        imgLocation.x = attrs.namedItem(BODY_JOINT_X_ATTR)
                .nodeValue().toInt();
        imgLocation.y = attrs.namedItem(BODY_JOINT_Y_ATTR)
                .nodeValue().toInt();
        imgLocation.x *= colsFactor;
        imgLocation.y *= rowsFactor;
        int id = attrs.namedItem(BODY_JOINT_ID_ATTR)
                .nodeValue().toInt();

        BodyJoint& joint = Utility::getJointById(bodyJoints, id);

        if( depthSign == "true" || depthSign == "1" ){
            joint.setDepthSign(true);
        } else{
            joint.setDepthSign(false);
        }
        joint.setImageLocation(imgLocation);
    });
}

void Project::loadKeyframeBodyParts(const QDomElement &node, tree<BodyPart> &bodyParts){
    Utility::loadXmlPart( node, BODY_PARTS_TAG,
    [&bodyParts]( const QDomElement& currNode, int ){
        const QDomNamedNodeMap& attrs = currNode.attributes();

        QString isOccluded = attrs.namedItem(BODY_PART_IS_OCCLUD_ATTR)
                .nodeValue();
        int id = attrs.namedItem(BODY_PART_ID_ATTR)
                .nodeValue().toInt();

        BodyPart& bodyPart = Utility::getBodyPartById(bodyParts, id);

        if( isOccluded == "true" || isOccluded == "1" ){
            bodyPart.setIsOccluded(true);
        } else{
            bodyPart.setIsOccluded(false);
        }
    });
}

//TODO: [i] Name of skeleton - forget
//TODO: [i] Scaling of skeleton - ignoring value. Try scale img later.

