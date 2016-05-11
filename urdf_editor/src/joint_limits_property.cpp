
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
    item = manager_->addProperty(QtVariantPropertyManager::enumTypeId(), QTranslator::tr("Units"));
    item->setAttribute(Common::attributeStr(EnumNames), QStringList() << QTranslator::tr("rad") << QTranslator::tr("deg"));
    top_item_->addSubProperty(item);
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
    double lower, upper, velocity;

    lower = limits_->lower*(1.0-non_SI_units_) + radianToDegree(limits_->lower)*(non_SI_units_);
    upper = limits_->upper*(1.0-non_SI_units_) + radianToDegree(limits_->upper)*(non_SI_units_);
    velocity = limits_->velocity*(1.0-non_SI_units_) + radianToDegree(limits_->velocity)*(non_SI_units_);

    for (int i = 0; i < sub_items.length(); ++i)
    {
      item = static_cast<QtVariantProperty *>(sub_items[i]);
      name = item->propertyName();
      if (name == "Units")
        item->setValue(non_SI_units_);
      else if (name == "Lower")
        item->setValue(lower);
      else if (name == "Upper")
        item->setValue(upper);
      else if (name == "Effort")
        item->setValue(limits_->effort);
      else if (name == "Velocity")
        item->setValue(velocity);
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
    if (name == "Units") 
    {
      non_SI_units_ = val.toDouble();
      loadData();
    }
    else if (name == "Lower")
      limits_->lower = val.toDouble()*(1.0-non_SI_units_) + degreeToRadian(val.toDouble())*(non_SI_units_);
    else if (name == "Upper")
      limits_->upper = val.toDouble()*(1.0-non_SI_units_) + degreeToRadian(val.toDouble())*(non_SI_units_);
    else if (name == "Effort")
      limits_->effort = val.toDouble();
    else if (name == "Velocity")
      limits_->velocity = val.toDouble()*(1.0-non_SI_units_) + degreeToRadian(val.toDouble())*(non_SI_units_);

    emit JointLimitsProperty::valueChanged(property, val);
  }

  double JointLimitsProperty::radianToDegree(double val)
  {
    return val*(180/M_PI);
  }

  double JointLimitsProperty::degreeToRadian(double val)
  {
    return val*(M_PI/180);
  }
}
