#ifndef FILEPATHSTORAGE
#define FILEPATHSTORAGE

#include <QString>
#include <QHash>

struct FilePathStorage{
public:
    struct FilenamePath{
        QString imgPath;
        QString maskPath;
        QString camPath;
    };
public:
    QString projectFilename;
    QString projectFolder;
    QString imgFolderPath;
    QString maskFolderPath;
    QHash<int, FilenamePath> paths;
};

#endif // FILEPATHSTORAGE

