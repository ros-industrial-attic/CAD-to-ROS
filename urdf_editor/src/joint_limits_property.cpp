
#include <qttreepropertybrowser.h>
#include <qtvariantproperty.h>

#include <urdf_editor/joint_limits_property.h>
#include <urdf_editor/common.h>

#include <urdf_model/joint.h>


namespace urdf_editor
{
  JointLimitsProperty::JointLimitsProperty(urdf::JointLimitsSharedPtr limits): limits_(limits), manager_(new QtVariantPropertyManager()), factory_(new QtVariantEditorFactory())
  {
    loading_ = true;
    QtVariantProperty *item;

    QObject::connect(manager_, SIGNAL(valueChanged(QtProperty *, const QVariant &)),
              this, SLOT(onValueChanged(QtProperty *, const QVariant &)));

    top_item_ = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), tr("Limit"));
    item = manager_->addProperty(QVariant::Double, tr("Lower"));
    item->setValue(limits_->lower);
    item->setAttribute(Common::attributeStr(Decimals), 6);
    top_item_->addSubProperty(item);
    item = manager_->addProperty(QVariant::Double, tr("Upper"));
    item->setValue(limits_->upper);
    item->setAttribute(Common::attributeStr(Decimals), 6);
    top_item_->addSubProperty(item);
    item = manager_->addProperty(QVariant::Double, tr("Effort"));
    item->setValue(limits_->effort);
    item->setAttribute(Common::attributeStr(Minimum), 0.0);
    item->setAttribute(Common::attributeStr(Decimals), 6);
    top_item_->addSubProperty(item);
    item = manager_->addProperty(QVariant::Double, tr("Velocity"));
    item->setValue(limits_->velocity);
    item->setAttribute(Common::attributeStr(Minimum), 0.0);
    item->setAttribute(Common::attributeStr(Decimals), 6);
    top_item_->addSubProperty(item);

    loading_ = false;
  }

  JointLimitsProperty::~JointLimitsProperty()
  {
    delete manager_;
    delete factory_;
  }

  void JointLimitsProperty::loadData()
  {
    loading_ = true;
    QtVariantProperty *item;
    QString name;
    QList<QtProperty *> sub_items = top_item_->subProperties();
    for (int i = 0; i < sub_items.length(); ++i)
    {
      item = static_cast<QtVariantProperty *>(sub_items[i]);
      name = item->propertyName();
      if (name == "Lower")
        item->setValue(limits_->lower);
      else if (name == "Upper")
        item->setValue(limits_->upper);
      else if (name == "Effort")
        item->setValue(limits_->effort);
      else if (name == "Velocity")
        item->setValue(limits_->velocity);
    }
    loading_ = false;
  }

  void JointLimitsProperty::loadFactoryForManager(QtTreePropertyBrowserSharedPtr& property_editor)
  {
    property_editor->setFactoryForManager(manager_, factory_);
  }

  void JointLimitsProperty::onValueChanged(QtProperty *property, const QVariant &val)
  {
    if (loading_)
      return;

    QString name = property->propertyName();
    if (name == "Lower")
      limits_->lower = val.toDouble();
    else if (name == "Upper")
      limits_->upper = val.toDouble();
    else if (name == "Effort")
      limits_->effort = val.toDouble();
    else if (name == "Velocity")
      limits_->velocity = val.toDouble();

    emit JointLimitsProperty::valueChanged(property, val);
  }
}
