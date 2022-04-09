
#ifndef MULTI_PROB_MAP_DISPLAY_H
#define MULTI_PROB_MAP_DISPLAY_H

#include <OGRE/OgreTexture.h>
#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreVector3.h>

#include <nav_msgs/MapMetaData.h>
#include <ros/time.h>

#include <nav_msgs/OccupancyGrid.h>
#include <multi_map_server/MultiOccupancyGrid.h>

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

class MultiProbMapDisplay: public Display
{
Q_OBJECT
public:
  MultiProbMapDisplay();
  virtual ~MultiProbMapDisplay();

  virtual void onInitialize();
  virtual void reset();
  virtual void update( float wall_dt, float ros_dt );

protected Q_SLOTS:
  void updateTopic();
  void updateDrawUnder();


protected:

  virtual void onEnable();
  virtual void onDisable();

  virtual void subscribe();
  virtual void unsubscribe();

  void incomingMap(const multi_map_server::MultiOccupancyGrid::ConstPtr& msg);

  void clear();
  
  std::vector<Ogre::ManualObject*> manual_object_;
  std::vector<Ogre::TexturePtr> texture_;
  std::vector<Ogre::MaterialPtr> material_;  
  
  bool loaded_;

  std::string topic_;

  ros::Subscriber map_sub_;

  RosTopicProperty* topic_property_;
  Property* draw_under_property_;

  multi_map_server::MultiOccupancyGrid::ConstPtr updated_map_;
  multi_map_server::MultiOccupancyGrid::ConstPtr current_map_;
  boost::mutex mutex_;
  bool new_map_;
};

} 

 #endif
