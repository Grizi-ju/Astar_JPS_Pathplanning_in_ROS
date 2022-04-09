
#ifndef PROB_MAP_DISPLAY_H
#define PROB_MAP_DISPLAY_H

#include <OGRE/OgreTexture.h>
#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreVector3.h>

#include <nav_msgs/MapMetaData.h>
#include <ros/time.h>

#include <nav_msgs/OccupancyGrid.h>

#include "rviz/display.h"

namespace Ogre
{
class ManualObject;
}

namespace rviz
{

class FloatProperty;
class IntProperty;
class Property;
class QuaternionProperty;
class RosTopicProperty;
class VectorProperty;

class ProbMapDisplay: public Display
{
Q_OBJECT
public:
  ProbMapDisplay();
  virtual ~ProbMapDisplay();

  virtual void onInitialize();
  virtual void fixedFrameChanged();
  virtual void reset();
  virtual void update( float wall_dt, float ros_dt );

  float getResolution() { return resolution_; }
  int getWidth() { return width_; }
  int getHeight() { return height_; }
  Ogre::Vector3 getPosition() { return position_; }
  Ogre::Quaternion getOrientation() { return orientation_; }

protected Q_SLOTS:
  void updateAlpha();
  void updateTopic();
  void updateDrawUnder();


protected:

  virtual void onEnable();
  virtual void onDisable();

  virtual void subscribe();
  virtual void unsubscribe();

  void incomingMap(const nav_msgs::OccupancyGrid::ConstPtr& msg);

  void clear();

  void transformMap();

  Ogre::ManualObject* manual_object_;
  Ogre::TexturePtr texture_;
  Ogre::MaterialPtr material_;
  bool loaded_;

  std::string topic_;
  float resolution_;
  int width_;
  int height_;
  Ogre::Vector3 position_;
  Ogre::Quaternion orientation_;
  std::string frame_;

  ros::Subscriber map_sub_;

  RosTopicProperty* topic_property_;
  FloatProperty* resolution_property_;
  IntProperty* width_property_;
  IntProperty* height_property_;
  VectorProperty* position_property_;
  QuaternionProperty* orientation_property_;
  FloatProperty* alpha_property_;
  Property* draw_under_property_;

  nav_msgs::OccupancyGrid::ConstPtr updated_map_;
  nav_msgs::OccupancyGrid::ConstPtr current_map_;
  boost::mutex mutex_;
  bool new_map_;
};

} 

 #endif
