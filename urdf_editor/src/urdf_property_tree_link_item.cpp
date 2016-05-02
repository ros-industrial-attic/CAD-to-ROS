#include "urdf_editor/urdf_property_tree_link_item.h"
#include "urdf_editor/link_property.h"
#include "urdf/model.h"
#include "urdf_editor/urdf_property_tree.h"
#include "ros/ros.h"

namespace urdf_editor
{

  URDFPropertyTreeLinkItem::URDFPropertyTreeLinkItem(urdf::LinkSharedPtr link, QStringList &link_names):
    QTreeWidgetItem(URDFPropertyTree::Link),
    link_(link),
    link_names_(link_names),
    property_(new LinkProperty(link)),
    assigned_joint_(NULL)
  {
    setText(0, QString::fromStdString(link_->name));
    LinkProperty *test = new LinkProperty(link);
    QObject::connect(test, SIGNAL(linkNameChanged(LinkProperty*,QVariant)),
              this, SLOT(on_linkNameChanged(LinkProperty*,QVariant)));

    QObject::connect(property_.get(), SIGNAL(valueChanged()),
              this, SLOT(on_valueChanged()));
  }

  QTreeWidgetItem *URDFPropertyTreeLinkItem::parent() const
  {
    QTreeWidgetItem::parent();
  }

  LinkPropertySharedPtr URDFPropertyTreeLinkItem::getPropertyData()
  {
    return property_;
  }

  urdf::LinkSharedPtr URDFPropertyTreeLinkItem::getData()
  {
    return link_;
  }

  void URDFPropertyTreeLinkItem::assignJoint(URDFPropertyTreeJointItem *joint)
  {
    assigned_joint_ = joint;
  }

  URDFPropertyTreeJointItem *URDFPropertyTreeLinkItem::getAssignedJoint()
  {
    if (!assigned_joint_)
      ROS_ERROR("A joint has not been assigned to Link(%s).", link_->name.c_str());

    return assigned_joint_;
  }

  bool URDFPropertyTreeLinkItem::hasAssignedJoint()
  {
    return (assigned_joint_ == NULL ? false : true);
  }

  void URDFPropertyTreeLinkItem::on_linkNameChanged(LinkProperty *link, const QVariant &val)
  {
    Q_UNUSED(link)
    QString current_name = text(0);
    QString new_name = val.toString();
    setText(0, new_name);

    int idx = link_names_.indexOf(current_name);
    link_names_.replace(idx, new_name);
    emit linkNameChanged(this);
  }

  void URDFPropertyTreeLinkItem::on_valueChanged()
  {
    emit valueChanged();
  }
}


