
#include <urdf_editor/joint_safety_property.h>
#include <urdf_editor/common.h>


namespace urdf_editor
{
  JointSafetyProperty::JointSafetyProperty(boost::shared_ptr<urdf::JointSafety> safety): safety_(safety), manager_(new QtVariantPropertyManager()), factory_(new QtVariantEditorFactory())
  {
    loading_ = true;
    QtVariantProperty *item;

    QObject::connect(manager_, SIGNAL(valueChanged(QtProperty *, const QVariant &)),
              this, SLOT(onValueChanged(QtProperty *, const QVariant &)));

    top_item_ = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), tr("Safety Controller"));
    item = manager_->addProperty(QVariant::Double, tr("Soft Lower Limit"));
    item->setAttribute(Common::attributeStr(Decimals), 6);
    top_item_->addSubProperty(item);
    item = manager_->addProperty(QVariant::Double, tr("Soft Upper Limit"));
    item->setAttribute(Common::attributeStr(Decimals), 6);
    top_item_->addSubProperty(item);
    item = manager_->addProperty(QVariant::Double, tr("K Position"));
    item->setAttribute(Common::attributeStr(Decimals), 6);
    top_item_->addSubProperty(item);
    item = manager_->addProperty(QVariant::Double, tr("K Velocity"));
    item->setAttribute(Common::attributeStr(Decimals), 6);
    top_item_->addSubProperty(item);

    loading_ = false;
  }

  JointSafetyProperty::~JointSafetyProperty()
  {
    delete manager_;
    delete factory_;
  }

  void JointSafetyProperty::loadData()
  {
    loading_ = true;
    QtVariantProperty *item;
    QString name;
    QList<QtProperty *> sub_items = top_item_->subProperties();
    for (int i = 0; i < sub_items.length(); ++i)
    {
      item = static_cast<QtVariantProperty *>(sub_items[i]);
      name = item->propertyName();
      if (name == "Soft Lower Limit")
        item->setValue(safety_->soft_lower_limit);
      else if (name == "Soft Upper Limit")
        item->setValue(safety_->soft_upper_limit);
      else if (name == "K Position")
        item->setValue(safety_->k_position);
      else if (name == "K Velocity")
        item->setValue(safety_->k_velocity);
    }
    loading_ = false;
  }

  void JointSafetyProperty::loadFactoryForManager(boost::shared_ptr<QtTreePropertyBrowser> &property_editor)
  {
    property_editor->setFactoryForManager(manager_, factory_);
  }

  void JointSafetyProperty::onValueChanged(QtProperty *property, const QVariant &val)
  {
    if (loading_)
      return;

    QString name = property->propertyName();
    if (name == "Soft Lower Limit")
      safety_->soft_lower_limit = val.toDouble();
    else if (name == "Soft Upper Limit")
      safety_->soft_upper_limit = val.toDouble();
    else if (name == "K Position")
      safety_->k_position = val.toDouble();
    else if (name == "K Velocity")
      safety_->k_velocity = val.toDouble();

    emit JointSafetyProperty::valueChanged(property, val);
  }
}
