
#include <qttreepropertybrowser.h>
#include <qtvariantproperty.h>

#include <urdf_editor/joint_dynamics_property.h>
#include <urdf_editor/common.h>

#include <urdf_model/joint.h>


namespace urdf_editor
{
  JointDynamicsProperty::JointDynamicsProperty(urdf::JointDynamicsSharedPtr dynamics): dynamics_(dynamics), manager_(new QtVariantPropertyManager()), factory_(new QtVariantEditorFactory())
  {
    loading_ = true;
    QtVariantProperty *item;

    QObject::connect(manager_, SIGNAL(valueChanged(QtProperty *, const QVariant &)),
              this, SLOT(onValueChanged(QtProperty *, const QVariant &)));

    top_item_ = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), tr("Dynamics"));
    item = manager_->addProperty(QVariant::Double, tr("Damping"));
    item->setAttribute(Common::attributeStr(Minimum), 0.0);
    item->setAttribute(Common::attributeStr(Decimals), 6);
    top_item_->addSubProperty(item);
    item = manager_->addProperty(QVariant::Double, tr("Friction"));
    item->setAttribute(Common::attributeStr(Minimum), 0.0);
    item->setAttribute(Common::attributeStr(Decimals), 6);
    top_item_->addSubProperty(item);

    loading_ = false;
  }

  JointDynamicsProperty::~JointDynamicsProperty()
  {
    delete manager_;
    delete factory_;
  }

  void JointDynamicsProperty::loadData()
  {
    loading_ = true;
    QtVariantProperty *item;
    QString name;
    QList<QtProperty *> sub_items = top_item_->subProperties();
    for (int i = 0; i < sub_items.length(); ++i)
    {
      item = static_cast<QtVariantProperty *>(sub_items[i]);
      name = item->propertyName();
      if (name == "Damping")
        item->setValue(dynamics_->damping);
      else if (name == "Friction")
        item->setValue(dynamics_->friction);
    }
    loading_ = false;
  }

  void JointDynamicsProperty::loadFactoryForManager(QtTreePropertyBrowserSharedPtr& property_editor)
  {
    property_editor->setFactoryForManager(manager_, factory_);
  }

  void JointDynamicsProperty::onValueChanged(QtProperty *property, const QVariant &val)
  {
    if (loading_)
      return;

    QString name = property->propertyName();
    if (name == "Damping")
      dynamics_->damping = val.toDouble();
    else if (name == "Friction")
      dynamics_->friction = val.toDouble();

    emit JointDynamicsProperty::valueChanged(property, val);
  }
}
