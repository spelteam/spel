#ifndef PROJECT_H
#define PROJECT_H
// SPEL definitions
#include "predef.hpp"

#include <QObject>
#include <QDomDocument>
#include <QFutureWatcher>

#include <frame.hpp>
#include <solvlet.hpp>

#include <opencv2/imgcodecs/imgcodecs_c.h>

#include <memory>
#include <vector>
#include "filepathstorage.h"
#include "projectattrstorage.h"

using namespace SPEL;
using namespace std;

namespace posegui {

  //TODO:[!] Merge frame and paths into single data structure
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
    using FramePtr = std::unique_ptr < Frame > ;
  private:
    using SkeletonPtr = std::unique_ptr < Skeleton > ;
  public:
    Project(const Project&) = delete;
    Project& operator=(const Project&) = delete;
    Project(const Project&&) = delete;
    Project& operator=(const Project&&) = delete;
  private:
    explicit Project(QObject *parent = 0);
  signals:
    void create();
    void load();
    void close();

    void keyframeUpdated();
    public slots:
    //void interpolateFramesEvent();
    private slots :
      void loadProjectEvent();
    void closeProjectEvent();
  public:
    static Project& getInstance();

    void open(const QString& filename, QFutureWatcher<void>* watcher = nullptr);

    void save(QFutureWatcher<void>* watcher = nullptr);

    void save(const QString& filename, QFutureWatcher<void>* watcher = nullptr);

    const QString& getProjectFolder() const;

    std::vector<Frame*> getFrames() const;
    Frame* getFrame(int num) const;
    void setFrames(std::vector<Frame*> _frames);

    const std::vector<LimbLabel>* getLabels(int num) const;

    const FilePathStorage& getPaths() const;

    ProjectState getState() const;

    void exchangeAtKeyframe(int num);
    void exchangeAtInterpolation(int num);

    void interpolateFrames();
    void solveFrames();
  private:
    //open project helpers
    void setProjectFolder(const QString &filename);
    void setProjectFilename(const QString &filename);
  public:
    //thread
    QFutureWatcher<void> futureWatcher;
  private:
    std::vector<FramePtr> frames;
    std::vector<Solvlet> solve;
    SkeletonPtr skeleton;
    FilePathStorage projectPaths;
    ProjectAttrStorage settings;
    ProjectState currState = ProjectState::CLOSED;
  };

}

#endif // PROJECT_H

