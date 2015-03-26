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
        INVALID_PROJECT_STATE = 1 << 4,
    };
    enum class ProjectState{
        CLOSED,
        OPENED,
        LOADED
    };
private:
    using FramePtr = std::unique_ptr<Frame>;
private:
    using SkeletonPtr = std::unique_ptr<Skeleton>;
public:
    Project( const Project& ) = delete;
    Project& operator=(const Project&) = delete;
    Project( const Project&& ) = delete;
    Project& operator=( const Project&& ) = delete;
private:
    explicit Project(QObject *parent = 0);

signals:
    void create();
    Project::ErrorCode open( const QString& filename );
    void load();
    void close();
    void save();

    void keyframeUpdated();

public slots:
    //void interpolateFramesEvent();
private slots:
    Project::ErrorCode openProjectEvent( const QString& filename );
    void loadProjectEvent();
    void closeProjectEvent();

public:
    static Project& getInstance();

    const QString& getProjectFolder() const;

    std::vector<Frame*> getFrames() const;
    Frame* getFrame( int num ) const;

    const QHash<int, FilenamePath>& getPaths() const;

    const QString& getLastError() const;

    ProjectState getState() const;

    void exchangeAtKeyframe( int num );
    void exchangeAtInterpolation( int num );

    void interpolateFrames();

private:
    //loading skeleton
    Project::ErrorCode loadSkeleton(const QDomDocument &document);
    Project::ErrorCode loadHeaderJoints( QDomElement &elem, tree<BodyJoint> &joints);
    Project::ErrorCode loadHeaderParts( QDomElement &elem, tree<BodyPart> &bodyParts,
                                          const tree<BodyJoint> &checkJoints );

    //loading frames
    Project::ErrorCode loadFrames(const QDomDocument &document);
    Project::ErrorCode loadKeyframeJoints( QDomElement &elem, tree<BodyJoint> &joints,
                                             float colsFactor, float rowsFactor);
    Project::ErrorCode loadKeyframeParts( QDomElement &elem, tree<BodyPart> &bodyParts);
    //open project helpers
    void setProjectFolder( const QString &filename );
    Project::ErrorCode readProjectXml( const QString &filename, QDomDocument &document );
    Project::ErrorCode validateProjectXml( const QDomDocument &document );
    //build structure of skeleton
    Project::ErrorCode buildBodyPartTree(std::vector<BodyPart> &bodyList,
                                          tree<BodyPart> &bodyParts);
private:
    std::vector<FramePtr> frames;
    QHash<int, FilenamePath> paths;
    SkeletonPtr skeleton;
    QString projectFolder;
    ProjectState currState = ProjectState::CLOSED;
    QString lastError;
};

#endif // PROJECT_H

