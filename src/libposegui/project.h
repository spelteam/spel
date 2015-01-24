#ifndef PROJECT_H
#define PROJECT_H

#include <QObject>
#include <QString>
#include <QHash>
#include <QDomDocument>
#include <frame.hpp>

#include <memory>
#include <vector>
#include <functional>


struct FilenamePath{
    static QString imgFolderPath;
    static QString maskFolderPath;

    QString imgPath;
    QString maskPath;
    QString camPath;
};


class Project : public QObject
{
    Q_OBJECT
public:
    enum class ErrorCode{
        SUCCESS = 0,
        INVALID_PROJECT_STRUCTURE = 1 << 1,
        MISSING_FILE = 1 << 2,
        FILE_NOT_OPEN = 1 << 3,
    };
private:
    using FramePtr = std::unique_ptr<Frame>;
    using SkeletonPtr = std::unique_ptr<Skeleton>;
private:
    explicit Project(QObject *parent = 0);
public:
    Project( const Project& ) = delete;
    Project& operator=(const Project&) = delete;
    Project( const Project&& ) = delete;
    Project& operator=( const Project&& ) = delete;

signals:
    void create();
    Project::ErrorCode open( const QString& filename, QString* errMessage = nullptr );
    void load();
    void close();
    void save();

public slots:

private slots:
    Project::ErrorCode openProjectEvent( const QString& filename, QString* errMessage = nullptr );

public:
    static Project& getInstance();

    std::vector<Frame*> getFrames() const;

private:
    void loadSkeleton( const QDomDocument& document );
    void loadFrames( const QDomDocument& document );
    void loadKeyframeJoints( const QDomElement& node, tree<BodyJoint>& bodyJoints,
                             float colsFactor, float rowsFactor );
    void loadKeyframeBodyParts( const QDomElement& node, tree<BodyPart>& bodyParts );

private:
    std::vector<FramePtr> frames;
    QHash<int, FilenamePath> paths;
    SkeletonPtr skeleton;
    QString projectFolder;
};

#endif // PROJECT_H

