#ifndef PROJECTATTR_H
#define PROJECTATTR_H

#include <QString>

namespace project_attributes {
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

}

#endif // PROJECTATTR_H

