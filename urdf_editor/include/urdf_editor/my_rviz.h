#ifndef __MYRVIZ_H__
#define __MYRVIZ_H__

#include <QWidget>

#include <urdf_editor/urdf_types_ext.h>

#include <ros/node_handle.h>


namespace rviz
{
  class Display;
  class RenderPanel;
  class VisualizationManager;
}

namespace moveit_rviz_plugin
{
  class RobotStateDisplay;
}

namespace urdf_editor
{
  class MyRviz: public QWidget
  {
    Q_OBJECT
  public:
    MyRviz(QWidget *parent = 0);
    virtual ~MyRviz();

    bool loadRobot(urdf::ModelInterfaceSharedPtr robot_model);

    bool clear();

  private:
    rviz::VisualizationManager *manager_;
    rviz::RenderPanel *render_panel_;
    moveit_rviz_plugin::RobotStateDisplay *robot_display_;
    rviz::Display *grid_display_;
    ros::NodeHandle nh_;
  };
}

#endif // __MYRVIZ_H__
