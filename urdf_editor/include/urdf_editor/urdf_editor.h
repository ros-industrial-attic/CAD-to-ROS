#ifndef __URDF_EDITOR_H__
#define __URDF_EDITOR_H__

#include <QMainWindow>

#include <urdf_editor/property_types.h>


namespace Ui
{
class URDFEditor;
}

class URDFEditor : public QMainWindow
{
  Q_OBJECT

public:
  explicit URDFEditor(QWidget *parent = 0);
  ~URDFEditor();

private slots:
  void on_action_Open_triggered();

  void on_action_Save_triggered();

  void on_actionSave_As_triggered();

  void on_action_New_triggered();

  void on_actionE_xit_triggered();

private:
  Ui::URDFEditor *ui;
  QList<QString> link_names;
  QList<QString> joint_names;

  urdf_editor::URDFPropertySharedPtr urdf_tree_;
  QString file_path_;
};

#endif // __URDF_EDITOR_H__
