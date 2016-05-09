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

    void updateBaseLink(std::string base);

    bool clear();

  public slots:
    /**
     * @brief Turns the robot model visual-model components on/off
     * @param If True, turns visual display on. If False, turns visual display off.
     */
    void enableVisualization(bool b);

    /**
     * @brief Turns the robot collision-model display on/off.
     * @param If True, turns collision display on. If False, turns collision display off.
     */
    void enableCollisionVisualization(bool b);

    void onLinkVisibilityChanged(const QString &, const bool &);

  private:
    rviz::VisualizationManager *manager_;
    rviz::RenderPanel *render_panel_;
    moveit_rviz_plugin::RobotStateDisplay *robot_display_;
    rviz::Display *grid_display_;
    rviz::Display *tf_display_;
    ros::NodeHandle nh_;
  };
}

#endif // __MYRVIZ_H__
