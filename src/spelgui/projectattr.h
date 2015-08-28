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

}

#endif // PROJECTATTR_H

