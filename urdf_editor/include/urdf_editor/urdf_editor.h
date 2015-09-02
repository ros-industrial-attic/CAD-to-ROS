#ifndef URDF_EDITOR_H
#define URDF_EDITOR_H

#include <QMainWindow>
#include <QtCore>
#include <QtGui>
#include <QObject>
#include "urdf_editor/qtpropertybrowser/qtvariantproperty.h"
#include "urdf_editor/qtpropertybrowser/qtpropertymanager.h"
#include "urdf_editor/qtpropertybrowser/qteditorfactory.h"
#include "urdf_editor/qtpropertybrowser/qttreepropertybrowser.h"
#include "urdf_editor/joint_property.h"
#include "urdf_editor/link_property.h"
#include "urdf_editor/urdf_property.h"


//#include "my_rviz.h"

namespace Ui
{
class URDFEditor;
}

namespace rviz
{
class Display;
class RenderPanel;
class VisualizationManager;
}

class URDFEditor : public QMainWindow
{
  Q_OBJECT

public:
  explicit URDFEditor(QWidget *parent = 0);
  ~URDFEditor();

private:
  Ui::URDFEditor *ui;
  QList<QString> link_names;
  QList<QString> joint_names;

  urdf_editor::URDFPropertyPtr urdf_tree_;
  rviz::VisualizationManager *rviz_manager_;
  rviz::RenderPanel *rviz_panel_;
  //urdf_editor::MyRviz *rviz_;
};

#endif // URDF_EDITOR_H
