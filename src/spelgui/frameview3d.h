#ifndef FRAMEVIEW3D_H
#define FRAMEVIEW3D_H

// SPEL definitions
#include "predef.hpp"

#include <QGLWidget>

class FrameView3D : public QGLWidget
{
  Q_OBJECT
public:
  explicit FrameView3D(QWidget *parent = 0);
  ~FrameView3D();
signals:
  void xRotationChanged(int angle);
  void yRotationChanged(int angle);
  void zRotationChanged(int angle);
protected:
  void initializeGL() override;
  void paintGL() override;
  void resizeGL(int width, int height) override;
private:
  //temp
  int xRot;
  int yRot;
  int zRot;
};

#endif // FRAMEVIEW3D_H
