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
    property_(new urdf_editor::JointProperty(joint, link_names, joint_names)),
    name_(QString::fromStdString(joint->name))
  {
    updateDisplayText();

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

  QString URDFPropertyTreeJointItem::getName()
  {
    return name_;
  }

  QString URDFPropertyTreeJointItem::getParentLinkName()
  {
    return QString::fromStdString(joint_->parent_link_name);
  }

  QString URDFPropertyTreeJointItem::getChildLinkName()
  {
    return QString::fromStdString(joint_->child_link_name);
  }

  void URDFPropertyTreeJointItem::setParentLinkName(QString name)
  {
    property_->setParentLinkName(name);
  }

  urdf::JointSharedPtr URDFPropertyTreeJointItem::getData()
  {
    return joint_;
  }

  JointPropertySharedPtr URDFPropertyTreeJointItem::getProperty()
  {
    return property_;
  }

  void URDFPropertyTreeJointItem::loadProperty(boost::shared_ptr<QtTreePropertyBrowser> property_editor)
  {
    property_->loadProperty(property_editor);
  }

  void URDFPropertyTreeJointItem::updateDisplayText()
  {
    setText(0, name_);
  }

  void URDFPropertyTreeJointItem::on_jointNameChanged(JointProperty *joint, const QVariant &val)
  {
    Q_UNUSED(joint)
    QString current_name = name_;
    name_ = val.toString();

    updateDisplayText();

    emit jointNameChanged(this, current_name, name_);
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
