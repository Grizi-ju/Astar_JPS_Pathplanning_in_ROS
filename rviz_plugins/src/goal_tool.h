
#ifndef RVIZ_GOAL_TOOL_H
#define RVIZ_GOAL_TOOL_H

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <QObject>

# include <ros/ros.h>

# include "pose_tool.h"
#endif

namespace rviz
{
class Arrow;
class DisplayContext;
class StringProperty;

class Goal3DTool: public Pose3DTool
{
Q_OBJECT
public:
  Goal3DTool();
  virtual ~Goal3DTool() {}
  virtual void onInitialize();

protected:
  virtual void onPoseSet(double x, double y, double z, double theta);

private Q_SLOTS:
  void updateTopic();

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;

  StringProperty* topic_property_;
};

}

#endif


