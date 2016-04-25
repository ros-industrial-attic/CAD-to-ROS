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

signals:
   void itemDropped(QTreeWidgetItem *);  

public:
  URDFPropertyTree(QWidget *parent);
  ~URDFPropertyTree() {}

private:
  virtual void dropEvent(QDropEvent * event);

};


#endif // URDF_PROPERTY_TREE_H