// SPEL definitions
#include "predef.hpp"

#include "frameview3d.h"

//PUBLIC

FrameView3D::FrameView3D(QWidget *parent)
  :QGLWidget(parent)
{

}

FrameView3D::~FrameView3D()
{

}

//PROTECTED

void FrameView3D::initializeGL(){
  return;
}

void FrameView3D::paintGL(){
  return;
}

void FrameView3D::resizeGL(int width, int height){
  return;
}

