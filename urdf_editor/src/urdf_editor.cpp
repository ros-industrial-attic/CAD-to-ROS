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

  ui->action_Save->setDisabled(true);

  urdf_tree_.reset(new urdf_editor::URDFProperty(ui->robotTreeWidget, ui->propertyBrowserContainer, ui->mainTabWidget->currentWidget()));
}

URDFEditor::~URDFEditor()
{
  delete ui;
}

void URDFEditor::on_action_Open_triggered()
{
  QString file_path = QFileDialog::getOpenFileName(this, tr("Open ROS URDF File"), QFileInfo(file_path_).dir().absolutePath(), tr("URDF Files (*.urdf)"));
  if (!file_path.isEmpty())
  {
    file_path_ = file_path;
    urdf_tree_->clear();
    urdf_tree_->loadURDF(file_path);
    ui->action_Save->setDisabled(false);
  }
}

void URDFEditor::on_action_Save_triggered()
{
  urdf_tree_->saveURDF(file_path_);
}

void URDFEditor::on_actionSave_As_triggered()
{
  QString file_path = QFileDialog::getSaveFileName(this, tr("Save As ROS URDF File"), QFileInfo(file_path_).dir().absolutePath(), tr("URDF Files (*.urdf)"));
  if (!file_path.isEmpty())
  {
    file_path_ = file_path;
    urdf_tree_->saveURDF(file_path);
    ui->action_Save->setDisabled(false);
  }
}

void URDFEditor::on_action_New_triggered()
{
    file_path_.clear();
    urdf_tree_->clear();
    ui->action_Save->setDisabled(true);
}

void URDFEditor::on_actionE_xit_triggered()
{
    QApplication::quit();
}
