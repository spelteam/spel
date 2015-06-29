#ifndef FRAMESHANDLER_H
#define FRAMESHANDLER_H

#include "modelhandler.h"
#include "bodyjointshandler.h"
#include "bodypartshandler.h"
#include "framehandler.h"

#include <vector>
#include <memory>

class QDomElement;
class QDomDocument;
struct FilePathStorage;
class Frame;
class Skeleton;
using FramePtr = std::unique_ptr<Frame>;
using Frames = std::vector<FramePtr>;
using SkeletonPtr = std::unique_ptr<Skeleton>;
namespace posegui{

class FramesHandler : public ModelHandler<Frames,QDomElement,QDomDocument,FilePathStorage,SkeletonPtr>
{
public:
    Frames read( const QDomElement &data, FilePathStorage &filepaths,
                 const SkeletonPtr &skeleton );
    QDomElement write( const Frames &model, const FilePathStorage &filepaths, QDomDocument &controller );
    virtual ~FramesHandler() override{}
private:
    BodyJointsHandler jointsHandler;
    BodyPartsHandler partsHandler;
    FrameHandler frameHandler;
};

}

#endif // FRAMESHANDLER_H
