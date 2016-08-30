
#include <urdf_editor/urdf_editor.h>
#include <urdf_editor/urdf_property.h>

#include <ui_industrial_robot_builder.h>

#include <rviz/render_panel.h>
#include <rviz/visualization_manager.h>

#include <QFileDialog>
#include <QMessageBox>
#include <QCloseEvent>

#include <qteditorfactory.h>
#include <qtpropertymanager.h>
#include <qttreepropertybrowser.h>
#include <qtvariantproperty.h>


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
  if (urdf_tree_->unsaved_changes_==false || (urdf_tree_->unsaved_changes_==true && unsaved_changes() ))
  {
    QString file_path = QFileDialog::getOpenFileName(this, tr("Open ROS URDF File"), QFileInfo(file_path_).dir().absolutePath(), tr("URDF Files (*.urdf *.xacro)"));
    if (!file_path.isEmpty())
    {
      file_path_ = file_path;
      urdf_tree_->clear();
      urdf_tree_->loadURDF(file_path);
      ui->action_Save->setDisabled(false);
    }
  }
}

bool URDFEditor::unsaved_changes()
{
  QMessageBox::StandardButton reply;
  reply = QMessageBox::question(this, "Save changes?", "Save your changes before closing?", QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel);

  if (reply == QMessageBox::Yes)
  {
    on_actionSave_As_triggered();
    return true;
  }
  if (reply == QMessageBox::Cancel)
    return false;
  if (reply == QMessageBox::No)
    return true;
}

void URDFEditor::on_action_Save_triggered()
{
  QMessageBox msgBox;
  if (urdf_tree_->saveURDF(file_path_))
  {
    urdf_tree_->unsaved_changes_ = false;
    msgBox.setWindowTitle("Success");
    msgBox.setText("The file was saved.");
    msgBox.exec();
  }
  else
  {
    msgBox.setWindowTitle("FAILURE");
    msgBox.setText("An error occurred during saving.");
    msgBox.exec();
  }
}

void URDFEditor::on_actionSave_As_triggered()
{
  QString file_path = QFileDialog::getSaveFileName(this, tr("Save As ROS URDF File"), QFileInfo(file_path_).dir().absolutePath(), tr("URDF Files (*.urdf)"));
  if (!file_path.isEmpty())
  {
    file_path_ = file_path;

    QMessageBox msgBox;
    if (!urdf_tree_->saveURDF(file_path))
    {
      msgBox.setWindowTitle("FAILURE");
      msgBox.setText("An error occurred during saving.");
      msgBox.exec();
    }
    else
    {
      urdf_tree_->unsaved_changes_ = false;
      ui->action_Save->setDisabled(false);
    }
  }
}

void URDFEditor::on_action_New_triggered()
{
  if (urdf_tree_->unsaved_changes_==false || (urdf_tree_->unsaved_changes_==true && unsaved_changes() ))
  {
    file_path_.clear();
    urdf_tree_->clear();
    ui->action_Save->setDisabled(true);
  }
}

void URDFEditor::on_action_Exit_triggered()
{
  emit close();
}

void URDFEditor::closeEvent( QCloseEvent *event )
{
  if ( urdf_tree_->unsaved_changes_ == true ) // If there were unsaved changes
  {
    if ( !unsaved_changes() ) // User canceled the quit
      event->ignore();
    else
    {
      event->accept();
    }
  }
  else
  {
    event->accept();
  }
}

void URDFEditor::on_actionToggle_Collision_Visualization_triggered(bool checked)
{
  ROS_DEBUG("Toggled collision-model display");
  urdf_tree_->requestCollisionVisualizationEnabled(checked);
}

void URDFEditor::on_action_ToggleVisual_triggered(bool checked)
{
  ROS_DEBUG("Toggled visual-model display");
  urdf_tree_->requestVisualizationEnabled(checked);
}

void URDFEditor::on_actionAbout_triggered()
{
  QMessageBox::about(this, "About", "Visualization toggle icons from https://icons8.com/.");
}
