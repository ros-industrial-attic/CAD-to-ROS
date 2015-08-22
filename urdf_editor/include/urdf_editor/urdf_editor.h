#ifndef URDF_EDITOR_H
#define URDF_EDITOR_H

#include <QMainWindow>
#include <QtCore>
#include <QtGui>
#include "include/urdf_editor/qtpropertybrowser/qtvariantproperty.h"
#include "include/urdf_editor/qtpropertybrowser/qtpropertymanager.h"
#include "include/urdf_editor/qtpropertybrowser/qteditorfactory.h"
#include "include/urdf_editor/qtpropertybrowser/qttreepropertybrowser.h"
#include "include/urdf_editor/joint_property.h"
#include "include/urdf_editor/link_property.h"
#include "include/urdf_editor/urdf_property.h"

namespace Ui {


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
