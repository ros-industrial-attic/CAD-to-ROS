#ifndef URDF_EDITOR_H
#define URDF_EDITOR_H

#include <QMainWindow>
#include <QtCore>
#include <QtGui>
#include <QTreeWidgetItem>
#include <urdf_parser/urdf_parser.h>
#include "include/urdf_editor/qtpropertybrowser/qtvariantproperty.h"
#include "include/urdf_editor/qtpropertybrowser/qtpropertymanager.h"
#include "include/urdf_editor/qtpropertybrowser/qteditorfactory.h"
#include "include/urdf_editor/qtpropertybrowser/qttreepropertybrowser.h"
#include "include/urdf_editor/joint_tree_property.h"
#include "include/urdf_editor/link_tree_property.h"

namespace Ui {


class URDFEditor;
}

class URDFEditor : public QMainWindow
{
  Q_OBJECT

public:
  explicit URDFEditor(QWidget *parent = 0);
  ~URDFEditor();

  void addLink();

  void addJoint(QTreeWidgetItem *parent);

  bool isJoint(QTreeWidgetItem *itm);

  QString getValidName(QString prefix, QList<QString> &current_names);

private slots:
  void on_robotTreeWidget_customContextMenuRequested(const QPoint &pos);

  void on_robotTreeWidget_itemClicked(QTreeWidgetItem *item, int column);

private:
  Ui::URDFEditor *ui;
  QTreeWidgetItem *root_;
  QTreeWidgetItem *links_;
  QTreeWidgetItem *joints_;
  QList<QString> link_names;
  QList<QString> joint_names;
  boost::shared_ptr<urdf::ModelInterface> model_;

  urdf_editor::JointTreeProperty *joint_tree_;
  urdf_editor::LinkTreeProperty *link_tree_;

  class QtTreePropertyBrowser *property_editor_;
  class QtVariantEditorFactory *variant_factory_;

};

#endif // URDF_EDITOR_H
