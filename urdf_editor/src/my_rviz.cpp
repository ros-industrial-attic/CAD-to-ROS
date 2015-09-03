#include "my_rviz.h"
#include <QVBoxLayout>

namespace urdf_editor
{

  MyRviz::MyRviz(QWidget *parent): QWidget(parent)
  {
    // Construct and layout render panel
    render_panel_ = new rviz::RenderPanel();
    QVBoxLayout *vlayout = new QVBoxLayout;
    vlayout->setMargin(0);
    vlayout->addWidget(render_panel_);

    // Set the top-level layout for this MyViz widget.
    setLayout( vlayout );

    // Next we initialize the main RViz classes.
    //
    // The VisualizationManager is the container for Display objects,
    // holds the main Ogre scene, holds the ViewController, etc.  It is
    // very central and we will probably need one in every usage of
    // librviz.
    manager_ = new rviz::VisualizationManager( render_panel_ );
    render_panel_->initialize( manager_->getSceneManager(), manager_ );
    manager_->initialize();
    manager_->startUpdate();
  }

  // Destructor
  MyRviz::~MyRviz()
  {
    delete manager_;
  }
}
