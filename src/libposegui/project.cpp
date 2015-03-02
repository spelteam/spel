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

#include <queue>

using namespace project_attributes;

//STATIC
QString FilenamePath::imgFolderPath = "";
QString FilenamePath::maskFolderPath = "";

//PUBLIC

Project& Project::getInstance(){
    static Project instance;
    return instance;
}

const QString& Project::getProjectFolder() const{
    return projectFolder;
}

std::vector<Frame*> Project::getFrames() const{
    std::vector<Frame*> framesPtr;

    for( const FramePtr& currFrame : frames ){
        framesPtr.push_back(currFrame.get());
    }

    return framesPtr;
}

Frame* Project::getFrame(int num) const{
    return frames.at(num).get();
}

const QHash<int, FilenamePath>& Project::getPaths() const{
    return paths;
}

const QString& Project::getLastError() const{
    return lastError;
}

Project::ProjectState Project::getState() const{
    return currState;
}



//PRIVATE

Project::Project(QObject *parent)
    :QObject(parent),
      frames(),
      paths(),
      skeleton( new Skeleton() ),
      projectFolder(),
      lastError()
{
    //connect
    QObject::connect(this, &Project::open, this, &Project::openProjectEvent);
    QObject::connect(this, &Project::load, this, &Project::loadProjectEvent);
    QObject::connect(this, &Project::close, this, &Project::closeProjectEvent);
}

#include <QDebug>
//TODO: [?]Go to background thread
//TODO: [L]Create status bar of loading
//TODO: [L] Error message write in low-level functions
//TODO: [!?]Refactor method
Project::ErrorCode Project::openProjectEvent(const QString &filename){
    if( currState == ProjectState::OPENED ){
        lastError = "Project is opened";
        return ErrorCode::INVALID_PROJECT_STATE;
    }
    ErrorCode error = ErrorCode::SUCCESS;

    //remember project's folder
    setProjectFolder( filename );
    //read content from file
    QDomDocument document;
    error = readProjectXml(filename, document);
    if( error != ErrorCode::SUCCESS ){
        close();
        return error;
    }
    //validate document
    error = validateProjectXml(document);
    if( error != ErrorCode::SUCCESS ){
        close();
        return error;
    }
    //read project options
    const QDomElement& projectElem = document.documentElement();
    const QDomNamedNodeMap& projectAttrs = projectElem.attributes();
    //set folder's path
    FilenamePath::imgFolderPath = projectAttrs.namedItem(PROJECT_IMG_FOLD_ATTR).nodeValue();
    FilenamePath::maskFolderPath = projectAttrs.namedItem(PROJECT_MASK_FOLD_ATTR).nodeValue();
    //set project options
    //TODO: [L]Setup project options
    //load skeleton
    error = loadSkeleton(document);
    if( error != ErrorCode::SUCCESS ){
        close();
        return error;
    }
    //load frames
    error = loadFrames(document);
    if( error != ErrorCode::SUCCESS ){
        close();
        return error;
    }
    //mark as opened
    currState = ProjectState::OPENED;

    return ErrorCode::SUCCESS;
}

void Project::closeProjectEvent(){
    //clear resources
    frames.clear();
    paths.clear();
    projectFolder.clear();
    //mark as closed
    currState = ProjectState::CLOSED;
}

//TODO: [!] This event is called last.
void Project::loadProjectEvent(){
    //mark as loaded
    qDebug() << "Project loading" << endl;
    currState = ProjectState::LOADED;

}

//UTILITIES FUNCTIONS

Project::ErrorCode Project::loadSkeleton(const QDomDocument &document){
    ErrorCode error = ErrorCode::SUCCESS;

    QDomElement jointTag = document
            .elementsByTagName(BODY_JOINTS_HEADER_TAG)
            .at(0)
            .firstChildElement();
    tree<BodyJoint> joints;
    error = loadHeaderJoints(jointTag, joints);
    if(error != ErrorCode::SUCCESS){
        lastError = "Couldn't load skeleton's joints structure";
        return error;
    }

    QDomElement bodyPartTag = document
            .elementsByTagName(BODY_PARTS_HEADER_TAG)
            .at(0)
            .firstChildElement();
    tree<BodyPart> bodyParts;
    error = loadHeaderParts(bodyPartTag, bodyParts, joints);
    if(error != ErrorCode::SUCCESS) return error;

    skeleton->setJointTree(joints);
    skeleton->setPartTree(bodyParts);
    skeleton->setScale(100.f);

    return ErrorCode::SUCCESS;
}

Project::ErrorCode Project::loadHeaderJoints(QDomElement &elem, tree<BodyJoint> &joints){
    tree<BodyJoint>::iterator jointsTop = joints.begin();

    while( !elem.isNull() ){
        //read info
        const QDomNamedNodeMap& attrs = elem.attributes();
        BodyJoint joint;

        int id = attrs.namedItem(BODY_JOINT_HEADER_ID_ATTR)
                .nodeValue()
                .toInt();
        QString name = attrs.namedItem(BODY_JOINT_HEADER_NAME_ATTR)
                .nodeValue();

        joint.setLimbID(id);
        joint.setJointName(name.toStdString());

        joints.insert( jointsTop, joint);
        //go to next joint
        elem = elem.nextSiblingElement();
    }

    return ErrorCode::SUCCESS;
}

Project::ErrorCode Project::loadHeaderParts(QDomElement &elem, tree<BodyPart> &bodyParts,
                                              const tree<BodyJoint> &checkJoints )
{
    std::vector<BodyPart> bodyList;
    while( !elem.isNull() ){
        //read info
        const QDomNamedNodeMap& attrs = elem.attributes();
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
        float relativeLength = attrs.namedItem(BODY_PART_HEADER_RELLENGTH_ATTR)
                .nodeValue().toFloat();
        //check whether child and parent joints are exist
        if( Utility::getJointById(checkJoints, parentJointId) == nullptr ||
            Utility::getJointById(checkJoints, childJointId) == nullptr )
        {
            lastError = "Value of joint doesn't exist";
            return ErrorCode::INVALID_PROJECT_STRUCTURE;
        }

        bodyPart.setPartID(id);
        bodyPart.setPartName(name.toStdString());
        bodyPart.setParentJoint(parentJointId);
        bodyPart.setChildJoint(childJointId);
        bodyPart.setLWRatio(lwRatio);
        bodyPart.setSpaceLength(expectedDistance);
        bodyPart.setRelativeLength(relativeLength);
        //add body part to list
        bodyList.push_back(bodyPart);
        //go to next body part
        elem = elem.nextSiblingElement();
    }
    //build skeleton's tree from list of body parts
    ErrorCode error = buildBodyPartTree(bodyList, bodyParts);
    if( error != ErrorCode::SUCCESS) return error;

    return ErrorCode::SUCCESS;
}

Project::ErrorCode Project::loadFrames(const QDomDocument &document){
    QDomElement frameTag = document
            .elementsByTagName(FRAMES_TAG)
            .at(0)
            .firstChildElement();
    //cols and rows of image in first occured frame
    int32_t firstFrameCol = -1;
    int32_t firstFrameRow = -1;
    //number of current frame
    int num = 0;
    while( !frameTag.isNull() ){
        //read info
        const QDomNamedNodeMap& attrs = frameTag.attributes();

        int id = attrs.namedItem(FRAME_ID_ATTR)
                .nodeValue().toInt();
        //load paths
        QString imgPath = attrs.namedItem(FRAME_IMG_PATH_ATTR).nodeValue();
        QString maskPath = attrs.namedItem(FRAME_MASK_PATH_ATTR).nodeValue();
        QString camPath = attrs.namedItem(FRAME_CAM_PATH_ATTR).nodeValue();
        //add new paths
        paths[num] = { imgPath, maskPath, camPath };
        //load image
        cv::Mat image = cv::imread(
            (projectFolder+FilenamePath::imgFolderPath+imgPath).toStdString(),
            CV_LOAD_IMAGE_COLOR
        );
        Q_ASSERT( image.data );
        //save original size of image
        int32_t imageOrigCols = image.cols;
        int32_t imageOrigRows = image.rows;
        Utility::resizeImage(image, firstFrameCol, firstFrameRow);
        //load mask
        cv::Mat mask = cv::imread(
            (projectFolder+FilenamePath::maskFolderPath+maskPath).toStdString(),
            CV_LOAD_IMAGE_GRAYSCALE
        );
        Q_ASSERT( mask.data );
        Utility::resizeImage(mask, firstFrameCol, firstFrameRow);
        //load ground point
        Point2f gp;
        gp.x = attrs.namedItem(FRAME_GPX_ATTR).nodeValue().toFloat();
        gp.y = attrs.namedItem(FRAME_GPY_ATTR).nodeValue().toFloat();

        QString isKeyframe = attrs.namedItem(FRAME_IS_KEY_ATTR)
                .nodeValue();
        //check whether it's keyframe
        FramePtr newFrame;
        if( isKeyframe == "true" || isKeyframe == "1" ){
            //create keyframe
            newFrame = FramePtr( new Keyframe() );
            //load keyframe info
            ErrorCode error = ErrorCode::SUCCESS;
            tree<BodyJoint> bodyJoints = skeleton->getJointTree();
            tree<BodyPart> bodyParts = skeleton->getPartTree();

            QDomElement jointTag = frameTag
                    .elementsByTagName(BODY_JOINTS_TAG)
                    .at(0)
                    .firstChildElement();
            float colsFactor = static_cast<float>(image.cols) / imageOrigCols;
            float rowsFactor = static_cast<float>(image.rows) / imageOrigRows;
            error = loadKeyframeJoints(jointTag, bodyJoints, colsFactor, rowsFactor);
            if(error != ErrorCode::SUCCESS){
                lastError = "Couldn't load keyframe's body joints structure";
                return error;
            }

            QDomElement bodyPartTag = frameTag
                    .elementsByTagName(BODY_PARTS_TAG)
                    .at(0)
                    .firstChildElement();
            error = loadKeyframeParts(bodyPartTag, bodyParts);
            if(error != ErrorCode::SUCCESS){
                lastError = "Couldn't load keyframe's body parts structure";
                return error;
            }

            Skeleton newSkeleton;
            newSkeleton.setJointTree(bodyJoints);
            newSkeleton.setPartTree(bodyParts);
            newSkeleton.setScale(100.f);

            newFrame->setSkeleton(newSkeleton);
        } else{
            //create interpolation frame
            newFrame = FramePtr( new Interpolation() );
            newFrame->setSkeleton( *skeleton.get() );
        }
        //write parameters to frame
        newFrame->setID(id);
        newFrame->setGroundPoint(gp);
        newFrame->setImage(image);
        newFrame->setMask(mask);
        //add frame to list
        frames.push_back( std::move(newFrame) );
        //go to next frame
        frameTag = frameTag.nextSiblingElement();
        ++num;
    }

    return ErrorCode::SUCCESS;
}

Project::ErrorCode Project::loadKeyframeJoints( QDomElement &elem, tree<BodyJoint> &joints,
                                                  float colsFactor, float rowsFactor)
{
    while( !elem.isNull() ){
        //read info
        const QDomNamedNodeMap& attrs = elem.attributes();

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

        BodyJoint* joint = Utility::getJointById(joints, id);
        //check whether it's joint exist
        if( joint == nullptr ) return ErrorCode::INVALID_PROJECT_STRUCTURE;

        joint->setDepthSign( depthSign == "true" || depthSign == "1" );
        joint->setImageLocation(imgLocation);
        //go to next joint
        elem = elem.nextSiblingElement();
    }

    return ErrorCode::SUCCESS;
}

Project::ErrorCode Project::loadKeyframeParts( QDomElement &elem, tree<BodyPart> &bodyParts){
    while ( !elem.isNull() ) {
        //read info
        const QDomNamedNodeMap& attrs = elem.attributes();

        QString isOccluded = attrs.namedItem(BODY_PART_IS_OCCLUD_ATTR)
                .nodeValue();
        int id = attrs.namedItem(BODY_PART_ID_ATTR)
                .nodeValue().toInt();

        BodyPart* bodyPart = Utility::getBodyPartById(bodyParts, id);
        if( bodyPart == nullptr ) return ErrorCode::INVALID_PROJECT_STRUCTURE;

        bodyPart->setIsOccluded( isOccluded == "true" || isOccluded == "1" );

        //go to next body part
        elem = elem.nextSiblingElement();
    }

    return ErrorCode::SUCCESS;
}

void Project::setProjectFolder(const QString &filename){
    QRegExp rx("^/.*/");
    rx.setMinimal(false);
    projectFolder = filename.split(
        filename.split(rx,QString::SkipEmptyParts).first(),
        QString::SkipEmptyParts
    ).first();
}

Project::ErrorCode Project::readProjectXml(const QString &filename, QDomDocument &document){
    QFile file(filename);
    if( file.open(QFile::ReadOnly) ){
        document.setContent(&file);
    } else{
        lastError = file.errorString();
        return ErrorCode::FILE_NOT_OPEN;
    }
    file.close();

    return ErrorCode::SUCCESS;
}

Project::ErrorCode Project::validateProjectXml(const QDomDocument &document){
    //TODO: [!]Magic const
    QFile file(":/root/resources/xml/project.xsd");
    QXmlSchema schema;
    XmlMessageHandler messagesHandler;
    schema.setMessageHandler(&messagesHandler);
    if( file.open(QFile::ReadOnly) ){
        schema.load(&file);
    } else{
        lastError = file.errorString();
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
        lastError = messagesHandler.errorMessage();
        return ErrorCode::INVALID_PROJECT_STRUCTURE;
    }

    return ErrorCode::SUCCESS;
}


Project::ErrorCode Project::buildBodyPartTree(std::vector<BodyPart> &bodyList,
                                              tree<BodyPart> &bodyParts)
{
    tree<BodyPart>::iterator root = bodyParts.begin();
    tree<BodyPart>::iterator currNode;
    bool isRootExist = false;
    //find root element
    for( auto it = bodyList.begin(); it != bodyList.end(); ++it ){
        if( it->getPartID() == BODY_PART_ROOT_ID ){
            currNode = bodyParts.insert( root, *it );
            bodyList.erase(it);
            isRootExist = true;
            break;
        }
    }
    if( !isRootExist ){
        lastError = "Root element of skeleton doesn't exist";
        return ErrorCode::INVALID_PROJECT_STRUCTURE;
    }
    std::queue<tree<BodyPart>::iterator> availableJoints;
    //add body parts to root element
    bodyList.erase( std::remove_if( bodyList.begin(), bodyList.end(),
    [&]( const BodyPart &bodyPart ){
        //check whether current body part is a child of root element
        if( currNode->getParentJoint() == bodyPart.getParentJoint() ||
                currNode->getChildJoint() == bodyPart.getParentJoint() )
        {
            //add body part to tree
            availableJoints.push(bodyParts.append_child(currNode, bodyPart));
            //remove current body part from list
            //and go next body part
            return true;
        } else{
            //go next body part
            return false;
        }
    }), bodyList.end() );
    //add body parts to rest part of tree
    while( !availableJoints.empty() ){
        //get current body part
       currNode = availableJoints.front();
       availableJoints.pop();
       //add body parts to tree
       //and remove from list
       bodyList.erase( std::remove_if(bodyList.begin(),bodyList.end(),
       [&]( const BodyPart &bodyPart ){
           if( currNode->getChildJoint() == bodyPart.getParentJoint() ){
               availableJoints.push(bodyParts.append_child(currNode, bodyPart));
               return true;
           } else{
               return false;
           }
       }), bodyList.end());
    }
    if( !bodyList.empty() ){
        lastError ="Some of body parts are invalid. Can't add to tree";
        return ErrorCode::INVALID_PROJECT_STRUCTURE;
    }
    return ErrorCode::SUCCESS;
}
//TODO: [i] Name of skeleton - forget
//TODO: [i] Scaling of skeleton - ignoring value. Try scale img later.

