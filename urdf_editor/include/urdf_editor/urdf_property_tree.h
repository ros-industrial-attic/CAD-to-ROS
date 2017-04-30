#ifndef URDF_PROPERTY_TREE_H
#define URDF_PROPERTY_TREE_H

#include <QApplication>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QDropEvent>

#include "urdf_editor/link_property.h"
#include "urdf_editor/joint_property.h"

class URDFPropertyTree: public QTreeWidget
{

  Q_OBJECT

public:
  URDFPropertyTree(QWidget *parent);
  ~URDFPropertyTree() {}

private slots:
  void on_itemPressed(QTreeWidgetItem *item, int column);

signals:
   void itemDropped(QTreeWidgetItem *);
   void dragDropEvent(QTreeWidgetItem *drag, QTreeWidgetItem *drop);

private:
  virtual void dropEvent(QDropEvent * event);

  QTreeWidgetItem *drag_item;
};


#endif // URDF_PROPERTY_TREE_H
