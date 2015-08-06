#include "projectattr.h"

namespace posegui {
  //body join header
  const QString BodyJointHeaderAttrs::JOINTS_TAG = "BodyJoints";
  const QString BodyJointHeaderAttrs::JOINT_TAG = "BodyJoint";
  const QString BodyJointHeaderAttrs::ID = "id";
  const QString BodyJointHeaderAttrs::NAME = "name";
  //body part header
  const QString BodyPartHeaderAttrs::PARTS_TAG = "BodyParts";
  const QString BodyPartHeaderAttrs::PART_TAG = "BodyPart";
  const QString BodyPartHeaderAttrs::ID = "id";
  const QString BodyPartHeaderAttrs::NAME = "name";
  const QString BodyPartHeaderAttrs::CHDJOINT = "childJointId";
  const QString BodyPartHeaderAttrs::PRTJOINT = "parentJointId";
  const QString BodyPartHeaderAttrs::LWRATIO = "lwRatio";
  const QString BodyPartHeaderAttrs::EXPECT_DIST = "expectedDistance";
  const QString BodyPartHeaderAttrs::REL_LENGTH = "relativeLength";
  const int BodyPartHeaderAttrs::ROOT_ID = 0;
  //body joint
  const QString BodyJointAttrs::JOINTS_TAG = "BodyJoints";
  const QString BodyJointAttrs::JOINT_TAG = "BodyJoint";
  const QString BodyJointAttrs::ID = "id";
  const QString BodyJointAttrs::X = "x";
  const QString BodyJointAttrs::Y = "y";
  const QString BodyJointAttrs::DEPTH = "depthSign";
  //body part
  const QString BodyPartAttrs::PARTS_TAG = "BodyParts";
  const QString BodyPartAttrs::PART_TAG = "BodyPart";
  const QString BodyPartAttrs::ID = "id";
  const QString BodyPartAttrs::IS_OCCLUDE = "isOccluded";
  //frame
  const QString FrameAttrs::FRAMES_TAG = "Frames";
  const QString FrameAttrs::FRAME_TAG = "Frame";
  const QString FrameAttrs::ID = "id";
  const QString FrameAttrs::IMG_PATH = "imgPath";
  const QString FrameAttrs::MASK_PATH = "maskPath";
  const QString FrameAttrs::CAM_PATH = "camPath";
  const QString FrameAttrs::GPX = "gpX";
  const QString FrameAttrs::GPY = "gpY";
  const QString FrameAttrs::IS_KEY = "isKeyframe";
  //project
  const QString ProjectAttrs::PROJECT_TAG = "Project";
  const QString ProjectAttrs::NAME = "name";
  const QString ProjectAttrs::IMG_FOLDER = "imgFolderPath";
  const QString ProjectAttrs::MASK_FOLDER = "maskFolderPath";
  const QString ProjectAttrs::CAM_FOLDER = "camFolderPath";
  const QString ProjectAttrs::ALLOW_SCALLING = "allowScaling";
  const QString ProjectAttrs::SIM_MAT_PATH = "simMatPath";
  const QString ProjectAttrs::EXPORT_PATH = "exportPath";
}
