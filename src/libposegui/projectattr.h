#ifndef PROJECTATTR_H
#define PROJECTATTR_H

#include <QString>

namespace posegui {

struct BodyJointHeaderAttrs{
    static const QString JOINTS_TAG;
    static const QString JOINT_TAG;
    static const QString ID;
    static const QString NAME;
};


struct BodyPartHeaderAttrs{
    static const QString PARTS_TAG;
    static const QString PART_TAG;
    static const QString ID;
    static const QString NAME;
    static const QString CHDJOINT;
    static const QString PRTJOINT;
    static const QString LWRATIO;
    static const QString EXPECT_DIST;
    static const QString REL_LENGTH;
    static const int ROOT_ID;
};

struct BodyJointAttrs{
    static const QString JOINTS_TAG;
    static const QString JOINT_TAG;
    static const QString ID;
    static const QString X;
    static const QString Y;
    static const QString DEPTH;
};

struct BodyPartAttrs{
    static const QString PARTS_TAG;
    static const QString PART_TAG;
    static const QString ID;
    static const QString IS_OCCLUDE;
};

struct FrameAttrs{
    static const QString FRAMES_TAG;
    static const QString FRAME_TAG;
    static const QString ID;
    static const QString IMG_PATH;
    static const QString MASK_PATH;
    static const QString CAM_PATH;
    static const QString GPX;
    static const QString GPY;
    static const QString IS_KEY;
};

struct ProjectAttrs{
    static const QString PROJECT_TAG;
    static const QString NAME;
    static const QString IMG_FOLDER;
    static const QString MASK_FOLDER;
    static const QString CAM_FOLDER;
    static const QString ALLOW_SCALLING;
    static const QString SIM_MAT_PATH;
    static const QString EXPORT_PATH;
};

//tags
const QString BODY_JOINTS_HEADER_TAG = "BodyJoints";
const QString BODY_JOINT_HEADER_TAG = "BodyJoint";
const QString BODY_PARTS_HEADER_TAG = "BodyParts";
const QString BODY_PART_HEADER_TAG = "BodyPart";
const QString FRAMES_TAG = "Frames";
const QString FRAME_TAG = "Frame";
const QString BODY_JOINTS_TAG = "BodyJoints";
const QString BODY_JOINT_TAG = "BodyJoint";
const QString BODY_PARTS_TAG = "BodyParts";
const QString BODY_PART_TAG = "BodyPart";
//Project attributes
const QString PROJECT_IMG_FOLD_ATTR = "imgFolderPath";
const QString PROJECT_MASK_FOLD_ATTR = "maskFolderPath";
//header BodyJoint attributes
const QString BODY_JOINT_HEADER_ID_ATTR = "id";
const QString BODY_JOINT_HEADER_NAME_ATTR = "name";
//header BodyPart attributes
const QString BODY_PART_HEADER_ID_ATTR = "id";
const QString BODY_PART_HEADER_NAME_ATTR = "name";
const QString BODY_PART_HEADER_CHDJOINT_ATTR = "childJointId";
const QString BODY_PART_HEADER_PRTJOINT_ATTR = "parentJointId";
const QString BODY_PART_HEADER_LWRATIO_ATTR = "lwRatio";
const QString BODY_PART_HEADER_EXPECTDIST_ATTR = "expectedDistance";
const QString BODY_PART_HEADER_RELLENGTH_ATTR = "relativeLength";
//frame attributes
const QString FRAME_ID_ATTR = "id";
const QString FRAME_IMG_PATH_ATTR = "imgPath";
const QString FRAME_MASK_PATH_ATTR = "maskPath";
const QString FRAME_CAM_PATH_ATTR = "camPath";
const QString FRAME_GPX_ATTR = "gpX";
const QString FRAME_GPY_ATTR = "gpY";
const QString FRAME_IS_KEY_ATTR = "isKeyframe";
//BodyJoint attributes
const QString BODY_JOINT_ID_ATTR = "id";
const QString BODY_JOINT_X_ATTR = "x";
const QString BODY_JOINT_Y_ATTR = "y";
const QString BODY_JOINT_DEPTH_ATTR = "depthSign";
//BodyPart attributes
const QString BODY_PART_ID_ATTR = "id";
const QString BODY_PART_IS_OCCLUD_ATTR = "isOccluded";
//root body part id
const int BODY_PART_ROOT_ID = 0;
}

#endif // PROJECTATTR_H

