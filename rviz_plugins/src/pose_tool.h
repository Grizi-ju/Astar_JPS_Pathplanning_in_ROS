
#ifndef RVIZ_POSE_TOOL_H
#define RVIZ_POSE_TOOL_H

#include <OGRE/OgreVector3.h>

#include <QCursor>

#include <ros/ros.h>

#include "rviz/tool.h"

namespace rviz
{
class Arrow;
class DisplayContext;

class Pose3DTool: public Tool
{
public:
  Pose3DTool();
  virtual ~Pose3DTool();

  virtual void onInitialize();

  virtual void activate();
  virtual void deactivate();

  virtual int processMouseEvent( ViewportMouseEvent& event );

protected:
  virtual void onPoseSet(double x, double y, double z, double theta) = 0;

  Arrow* arrow_;
  std::vector<Arrow*> arrow_array;

  enum State
  {
    Position,
    Orientation,
    Height
  };
  State state_;

  Ogre::Vector3 pos_;
};

}

#endif


