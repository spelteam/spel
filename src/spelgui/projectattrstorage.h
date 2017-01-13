#ifndef PROJECTATTRSTORAGE
#define PROJECTATTRSTORAGE

#include <QString>

struct ProjectAttrStorage{
    QString name;
    QString imgFolderPath;
    QString maskFolderPath;
    QString camFolderPath;
    bool allowScaling;
    QString simMatPath;
    QString exportPath;
};

#endif // PROJECTATTRSTORAGE

