#include "urdf_editor/my_rviz.h"
#include <urdf_parser/urdf_parser.h>
#include <QVBoxLayout>

namespace urdf_editor
{

  MyRviz::MyRviz(QWidget *parent): QWidget(parent)
  {
    // Construct and layout render panel
    render_panel_ = new rviz::RenderPanel();
    QVBoxLayout *vlayout = new QVBoxLayout(parent);
    vlayout->setMargin(0);
    vlayout->addWidget(render_panel_);

    manager_ = new rviz::VisualizationManager( render_panel_ );
    render_panel_->initialize( manager_->getSceneManager(), manager_ );
    manager_->initialize();
    manager_->startUpdate();
    render_panel_->setEnabled(true);
    render_panel_->setBackgroundColor(Ogre::ColourValue(0.25, 0.25, 0.25, 1));

    robot_display_ = new moveit_rviz_plugin::RobotStateDisplay();
    robot_display_->setName("RobotModel");
    robot_display_->subProp("Robot Description")->setValue("/ros_workbench");
    manager_->addDisplay(robot_display_, false);

    grid_display_ = manager_->createDisplay("rviz/Grid", "MyGrid", true);
    grid_display_->subProp("Line Style")->setValue("Billboards");
  }

  // Destructor
  MyRviz::~MyRviz()
  {
    delete manager_;
  }

  bool MyRviz::loadRobot(boost::shared_ptr<urdf::ModelInterface> robot_model)
  {
    robot_display_->setEnabled(false);
    ros::NodeHandle nh("~");
    TiXmlDocument *robot_document = urdf::exportURDF(robot_model);
    TiXmlPrinter printer;
    robot_document->Accept(&printer);
    nh.setParam("/ros_workbench", printer.CStr());
    robot_display_->reset();
    robot_display_->setEnabled(true);
  }

}
