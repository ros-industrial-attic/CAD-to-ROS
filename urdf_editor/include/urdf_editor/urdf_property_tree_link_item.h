#ifndef URDF_PROPERTY_TREE_LINK_ITEM_H
#define URDF_PROPERTY_TREE_LINK_ITEM_H

#include <QTreeWidgetItem>
#include <QVariant>

#include "urdf_editor/urdf_types.h"
#include "urdf_editor/property_types.h"
#include "urdf_editor/link_property.h"
#include "urdf_editor/urdf_property_tree_joint_item.h"

namespace urdf_editor
{
  class URDFPropertyTreeLinkItem: public QObject, public QTreeWidgetItem
  {
    Q_OBJECT
  public:
    explicit URDFPropertyTreeLinkItem(urdf::LinkSharedPtr link, QStringList &link_names);
    ~URDFPropertyTreeLinkItem() {}

    QTreeWidgetItem *parent() const;
    LinkPropertySharedPtr getPropertyData();
    urdf::LinkSharedPtr getData();

    void assignJoint(URDFPropertyTreeJointItem *joint);
    URDFPropertyTreeJointItem *getAssignedJoint();
    bool hasAssignedJoint();

  private slots:
    void on_linkNameChanged(LinkProperty *link, const QVariant &val);
    void on_valueChanged();

  signals:
    void valueChanged();
    void linkNameChanged(URDFPropertyTreeLinkItem *link);

  private:
    urdf::LinkSharedPtr link_;
    LinkPropertySharedPtr property_;
    QStringList &link_names_;
    URDFPropertyTreeJointItem* assigned_joint_;

  };
}

#endif // URDF_PROPERTY_TREE_LINK_ITEM_H
