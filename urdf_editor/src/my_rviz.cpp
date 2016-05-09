
#include <urdf_editor/my_rviz.h>

#include <QVBoxLayout>
#include <QtCore>

#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>

#include <moveit/robot_state_rviz_plugin/robot_state_display.h>

#include <urdf_parser/urdf_parser.h>
#include <urdf/model.h>


namespace urdf_editor
{
  MyRviz::MyRviz(QWidget *parent): QWidget(parent), nh_("~")
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
    robot_display_->subProp("Robot Description")->setValue("ros_workbench");
    manager_->addDisplay(robot_display_, false);

    grid_display_ = manager_->createDisplay("rviz/Grid", "MyGrid", true);

    tf_display_ = manager_->createDisplay("rviz/TF", "TF", true);
    tf_display_->subProp("Show Arrows")->setValue("true");
    tf_display_->subProp("Show Names")->setValue("true");
    tf_display_->subProp("Show Axes")->setValue("true");
    tf_display_->subProp("Frame Timeout")->setValue("1");
  }

  // Destructor
  MyRviz::~MyRviz()
  {
    delete manager_;
  }

  bool MyRviz::loadRobot(urdf::ModelInterfaceSharedPtr robot_model)
  {
    robot_display_->setEnabled(false);
    TiXmlDocument *robot_document = urdf::exportURDF(robot_model);
    TiXmlPrinter printer;
    robot_document->Accept(&printer);
    nh_.setParam("ros_workbench", printer.CStr());
    robot_display_->reset();
    robot_display_->setEnabled(true);

    // TODO: return result of serialisation
    return true;
  }

  void MyRviz::updateBaseLink(std::string base)
  {
    QString base_link = QString::fromStdString(base);
    manager_->setFixedFrame(base_link);
  }

  bool MyRviz::clear()
  {
    robot_display_->setEnabled(false);
    nh_.deleteParam("ros_workbench");
    robot_display_->reset();
    robot_display_->setEnabled(true);

    // TODO: return result of parameter deletion
    return true;
  }

  void MyRviz::enableVisualization(bool b)
  {
    robot_display_->subProp("Visual Enabled")->setValue (b ? "true" : "false");
  }

  void MyRviz::enableCollisionVisualization(bool b)
  {
    robot_display_->subProp("Collision Enabled")->setValue (b ? "true" : "false");
  }

  void MyRviz::onLinkVisibilityChanged(const QString& link_name, const bool& value)
  {
    // TODO: this assumes that 'link_name' exists and is a sub property of the
    //       robot_display. rviz::Property::subProp(..) cannot fail (it returns
    //       the FailureProperty instance if a property doesn't exist), but
    //       doing our own checking is probably nicer.
    robot_display_->subProp("Links")->subProp(link_name)->setValue(value);
  }
}
