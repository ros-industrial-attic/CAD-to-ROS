#include "urdf_editor/urdf_property_tree_joint_item.h"
#include "urdf_editor/joint_property.h"
#include "urdf/model.h"
#include "urdf_editor/urdf_property_tree.h"

namespace urdf_editor
{

  URDFPropertyTreeJointItem::URDFPropertyTreeJointItem(urdf::JointSharedPtr joint, QStringList &link_names, QStringList &joint_names):
    QTreeWidgetItem(URDFPropertyTree::Joint),
    joint_(joint),
    link_names_(link_names),
    joint_names_(joint_names),
    property_(new urdf_editor::JointProperty(joint, link_names, joint_names))
  {
    setText(0, QString::fromStdString(joint_->name));
    QObject::connect(property_.get(), SIGNAL(jointNameChanged(JointProperty *, const QVariant &)),
              this, SLOT(on_jointNameChanged(JointProperty*,const QVariant &)));
    QObject::connect(property_.get(), SIGNAL(parentLinkChanged(JointProperty *, const QVariant &)),
              this, SLOT(on_jointParentLinkChanged(JointProperty*,QVariant)));
    QObject::connect(property_.get(), SIGNAL(valueChanged()),
              this, SLOT(on_valueChanged()));
  }

  QTreeWidgetItem *URDFPropertyTreeJointItem::parent() const
  {
    return QTreeWidgetItem::parent();
  }

  JointPropertySharedPtr URDFPropertyTreeJointItem::getPropertyData()
  {
    return property_;
  }

  urdf::JointSharedPtr URDFPropertyTreeJointItem::getData()
  {
    return joint_;
  }

  void URDFPropertyTreeJointItem::on_jointNameChanged(JointProperty *joint, const QVariant &val)
  {
    Q_UNUSED(joint)
    QString current_name = text(0);
    QString new_name = val.toString();
    setText(0, new_name);

    int idx = joint_names_.indexOf(current_name);
    joint_names_.replace(idx, new_name);
    emit jointNameChanged(this);
  }

  void URDFPropertyTreeJointItem::on_jointParentLinkChanged(JointProperty *joint, const QVariant &val)
  {
    emit parentLinkChanged(this);
  }

  void URDFPropertyTreeJointItem::on_valueChanged()
  {
    emit valueChanged();
  }
}
