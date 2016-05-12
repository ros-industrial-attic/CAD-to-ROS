#ifndef __URDF_PROPERTY_TREE_LINK_ITEM_H__
#define __URDF_PROPERTY_TREE_LINK_ITEM_H__

#include <QTreeWidgetItem>
#include <urdf_editor/urdf_types.h>
#include <urdf_editor/property_types.h>
#include <urdf_editor/link_property.h>
#include <urdf_editor/urdf_property_tree_joint_item.h>

namespace urdf_editor
{
  class URDFPropertyTreeLinkItem: public QObject, public QTreeWidgetItem
  {
    Q_OBJECT
  public:
    explicit URDFPropertyTreeLinkItem(urdf::LinkSharedPtr link, QStringList &link_names);
    virtual ~URDFPropertyTreeLinkItem() {}

    QTreeWidgetItem *parent() const;

    QString getName();
    urdf::LinkSharedPtr getData();
    LinkPropertySharedPtr getProperty(); // this method should be removed once the property menu is moved to their respected propery class.
    void loadProperty(boost::shared_ptr<QtTreePropertyBrowser> property_editor);

    void assignJoint(URDFPropertyTreeJointItem *joint);
    void unassignJoint();
    URDFPropertyTreeJointItem *getAssignedJoint();
    bool hasAssignedJoint();

  private slots:
    void on_linkNameChanged(LinkProperty *link, const QVariant &val);
    void on_valueChanged();

  signals:
    void valueChanged();
    void linkNameChanged(URDFPropertyTreeLinkItem *link, QString current_name, QString new_name);

  private:
    void updateDisplayText();

    QString name_;
    urdf::LinkSharedPtr link_;
    LinkPropertySharedPtr property_;
    QStringList &link_names_;
    URDFPropertyTreeJointItem* assigned_joint_;

  };
}

#endif // __URDF_PROPERTY_TREE_LINK_ITEM_H__
