#include "urdf_editor/urdf_editor.h"
#include "ui_industrial_robot_builder.h"
#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include <sstream>
#include <QFileDialog>

URDFEditor::URDFEditor(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::URDFEditor)
{
  ui->setupUi(this);

  QString file_path = QFileDialog::getOpenFileName(this,tr("Open ROS URDF File"),"",tr("URDF Files (*.urdf)"));

  urdf_tree_.reset(new urdf_editor::URDFProperty(ui->robotTreeWidget, ui->propertyBrowserContainer, ui->mainTabWidget->currentWidget()));
  urdf_tree_->loadURDF(file_path);
}

URDFEditor::~URDFEditor()
{
  delete ui;
}


