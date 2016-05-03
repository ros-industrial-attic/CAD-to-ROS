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
    assigned_joint_(NULL),
    name_(QString::fromStdString(link->name))
  {
    updateDisplayText();

    QObject::connect(property_.get(), SIGNAL(linkNameChanged(LinkProperty*,QVariant)),
              this, SLOT(on_linkNameChanged(LinkProperty*,QVariant)));

    QObject::connect(property_.get(), SIGNAL(valueChanged()),
              this, SLOT(on_valueChanged()));
  }

  QTreeWidgetItem *URDFPropertyTreeLinkItem::parent() const
  {
    QTreeWidgetItem::parent();
  }

  QString URDFPropertyTreeLinkItem::getName()
  {
    return name_;
  }

  urdf::LinkSharedPtr URDFPropertyTreeLinkItem::getData()
  {
    return link_;
  }

  LinkPropertySharedPtr URDFPropertyTreeLinkItem::getProperty()
  {
    return property_;
  }

  void URDFPropertyTreeLinkItem::loadProperty(boost::shared_ptr<QtTreePropertyBrowser> property_editor)
  {
    property_->loadProperty(property_editor);
  }

  void URDFPropertyTreeLinkItem::assignJoint(URDFPropertyTreeJointItem *joint)
  {
    assigned_joint_ = joint;

    //Should we be keeping all data within the urdf::Link up to date?
    link_->parent_joint = assigned_joint_->getData();
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

  void URDFPropertyTreeLinkItem::updateDisplayText()
  {
    setText(0, name_);
  }

  void URDFPropertyTreeLinkItem::on_linkNameChanged(LinkProperty *link, const QVariant &val)
  {
    Q_UNUSED(link)
    QString current_name = name_;
    name_ = val.toString();

    updateDisplayText();

    emit linkNameChanged(this, current_name, name_);
  }

  void URDFPropertyTreeLinkItem::on_valueChanged()
  {
    emit valueChanged();
  }
}


