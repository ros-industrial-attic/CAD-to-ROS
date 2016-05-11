
#include <qttreepropertybrowser.h>
#include <qtvariantproperty.h>

#include <urdf_editor/joint_axis_property.h>
#include <urdf_editor/common.h>

#include <urdf_model/pose.h>


namespace urdf_editor
{
  JointAxisProperty::JointAxisProperty(urdf::Vector3 &axis): axis_(axis), manager_(new QtVariantPropertyManager()), factory_(new QtVariantEditorFactory())
  {
    loading_ = true;
    QtVariantProperty *item;

    QObject::connect(manager_, SIGNAL(valueChanged(QtProperty *, const QVariant &)),
              this, SLOT(onValueChanged(QtProperty *, const QVariant &)));

    top_item_ = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), tr("Axis"));
    item = manager_->addProperty(QtVariantPropertyManager::enumTypeId(), QTranslator::tr("Units"));
    item->setAttribute(Common::attributeStr(EnumNames), QStringList() << QTranslator::tr("m") << QTranslator::tr("in"));
    top_item_->addSubProperty(item);
    item = manager_->addProperty(QVariant::Double, tr("X"));
    item->setAttribute(Common::attributeStr(Decimals), 6);
    top_item_->addSubProperty(item);
    item = manager_->addProperty(QVariant::Double, tr("Y"));
    item->setAttribute(Common::attributeStr(Decimals), 6);
    top_item_->addSubProperty(item);
    item = manager_->addProperty(QVariant::Double, tr("Z"));
    item->setAttribute(Common::attributeStr(Decimals), 6);
    top_item_->addSubProperty(item);

    loading_ = false;
  }

  JointAxisProperty::~JointAxisProperty()
  {
    delete manager_;
    delete factory_;
  }

  void JointAxisProperty::loadData()
  {
    loading_ = true;
    QtVariantProperty *item;
    QString name;
    double x_pos, y_pos, z_pos;
    QList<QtProperty *> sub_items = top_item_->subProperties();
    for (int i = 0; i < sub_items.length(); ++i)
    {
      item = static_cast<QtVariantProperty *>(sub_items[i]);
      name = item->propertyName();
      if (name == "Units")
        item->setValue(non_SI_units_);
      else if (name == "X") 
      {
        x_pos = axis_.x*(1.0-non_SI_units_) + meterToInch(axis_.x)*(non_SI_units_);
        item->setValue(x_pos);
      }
      else if (name == "Y")
      {
        y_pos = axis_.y*(1.0-non_SI_units_) + meterToInch(axis_.y)*(non_SI_units_);
        item->setValue(y_pos);
      }
      else if (name == "Z")
      {
        z_pos = axis_.z*(1.0-non_SI_units_) + meterToInch(axis_.z)*(non_SI_units_);
        item->setValue(z_pos);
      }
    }
    loading_ = false;
  }

  void JointAxisProperty::loadFactoryForManager(QtTreePropertyBrowserSharedPtr& property_editor)
  {
    property_editor->setFactoryForManager(manager_, factory_);
  }

  void JointAxisProperty::onValueChanged(QtProperty *property, const QVariant &val)
  {
    if (loading_)
      return;

    QString name = property->propertyName();
    if (name == "Units") 
    {
      non_SI_units_ = val.toDouble();
      loadData();
    }
    else if (name == "X")
      axis_.x =  val.toDouble()*(1.0-non_SI_units_) + inchToMeter(val.toDouble())*(non_SI_units_);
    else if (name == "Y")
      axis_.y = val.toDouble()*(1.0-non_SI_units_) + inchToMeter(val.toDouble())*(non_SI_units_);
    else if (name == "Z")
      axis_.z = val.toDouble()*(1.0-non_SI_units_) + inchToMeter(val.toDouble())*(non_SI_units_);

    emit JointAxisProperty::valueChanged(property, val);
  }

  double JointAxisProperty::meterToInch(double val)
  {
    return val/0.0254;
  }

  double JointAxisProperty::inchToMeter(double val)
  {
    return val*0.0254;
  }
}
