#include "include/urdf_editor/urdf_editor.h"
#include "ui_industrial_robot_builder.h"
#include <sstream>

URDFEditor::URDFEditor(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::URDFEditor)
{
  ui->setupUi(this);

  QString file_path = "/home/larmstrong/catkin_iiwa_ws/src/kuka_experimental/kuka_lbr_iiwa_support/urdf/lbr_iiwa_14_r820.urdf";

  urdf_tree_.reset(new urdf_editor::URDFProperty(ui->robotTreeWidget, ui->propertyBrowserContainer));
  urdf_tree_->loadURDF(file_path);
}

URDFEditor::~URDFEditor()
{
  delete ui;
}


