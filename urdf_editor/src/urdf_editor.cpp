#include "include/urdf_editor/urdf_editor.h"
#include "ui_industrial_robot_builder.h"
#include <sstream>

URDFEditor::URDFEditor(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::URDFEditor)
{
  ui->setupUi(this);

  QString file_path = "/home/larmstrong/catkin_abb_ws/src/abb/abb_irb2400_support/urdf/irb2400.urdf";

  urdf_tree_.reset(new urdf_editor::URDFProperty(ui->robotTreeWidget, ui->propertyBrowserContainer));
  urdf_tree_->loadURDF(file_path);
}

URDFEditor::~URDFEditor()
{
  delete ui;
}


