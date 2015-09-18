#ifndef URDF_EDITOR_H
#define URDF_EDITOR_H

#include <QMainWindow>
#include <QtCore>
#include <QtGui>
#include <QObject>
#include <qtvariantproperty.h>
#include <qtpropertymanager.h>
#include <qteditorfactory.h>
#include <qteditorfactory.h>
#include <qttreepropertybrowser.h>
#include "urdf_editor/joint_property.h"
#include "urdf_editor/link_property.h"
#include "urdf_editor/urdf_property.h"
#include "my_rviz.h"

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

private:
  Ui::URDFEditor *ui;
  QList<QString> link_names;
  QList<QString> joint_names;

  urdf_editor::URDFPropertyPtr urdf_tree_;
};

#endif // URDF_EDITOR_H
