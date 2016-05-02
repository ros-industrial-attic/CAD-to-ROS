#ifndef URDF_PROPERTY_TREE_JOINT_ITEM_H
#define URDF_PROPERTY_TREE_JOINT_ITEM_H

#include <QTreeWidgetItem>
#include <QVariant>

#include "urdf_editor/urdf_types.h"
#include "urdf_editor/property_types.h"
#include "urdf_editor/joint_property.h"

namespace urdf_editor
{

  class URDFPropertyTreeJointItem: public QObject, public QTreeWidgetItem
  {
    Q_OBJECT
  public:
    explicit URDFPropertyTreeJointItem(urdf::JointSharedPtr joint, QStringList &link_names, QStringList &joint_names);
    ~URDFPropertyTreeJointItem() {}

    QTreeWidgetItem *parent() const;
    JointPropertySharedPtr getPropertyData();
    urdf::JointSharedPtr getData();

  private slots:
    void on_jointNameChanged(JointProperty *joint, const QVariant &val);
    void on_jointParentLinkChanged(JointProperty *joint, const QVariant &val);
    void on_valueChanged();

  signals:
    void valueChanged();
    void jointNameChanged(URDFPropertyTreeJointItem *joint);
    void parentLinkChanged(URDFPropertyTreeJointItem *joint);

  private:
    urdf::JointSharedPtr joint_;
    QStringList &link_names_;
    QStringList &joint_names_;
    JointPropertySharedPtr property_;

  };
}

#endif // URDF_PROPERTY_TREE_JOINT_ITEM_H
