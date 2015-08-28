#ifndef FRAMEATTRSTORAGE
#define FRAMEATTRSTORAGE

#include <QString>

struct FrameAttrStorage{
    int id;
    QString imgPath;
    QString maskPath;
    QString camPath;
    float gpX;
    float gpY;
    bool isKeyframe;
};

#endif // FRAMEATTRSTORAGE

