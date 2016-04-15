#ifndef MYRVIZ_H
#define MYRVIZ_H

#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include <moveit/robot_state_rviz_plugin/robot_state_display.h>
#include <urdf/model.h>

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

    bool loadRobot(boost::shared_ptr<urdf::ModelInterface> robot_model);
    
    moveit_rviz_plugin::RobotStateDisplay* getRobotDisplay();

    bool clear();

  private:
    rviz::VisualizationManager *manager_;
    rviz::RenderPanel *render_panel_;
    moveit_rviz_plugin::RobotStateDisplay *robot_display_;
    rviz::Display *grid_display_;
    ros::NodeHandle nh_;
  };

}

#endif // MYRVIZ_H
