
#include <qttreepropertybrowser.h>
#include <qtvariantproperty.h>

#include <urdf_editor/origin_property.h>
#include <urdf_editor/common.h>

#include <urdf_model/pose.h>
#include <iostream>


namespace urdf_editor
{
  OriginProperty::OriginProperty(urdf::Pose &origin): origin_(origin), manager_(new QtVariantPropertyManager()), factory_(new QtVariantEditorFactory())
  {
    loading_ = true;
    QtVariantProperty *item;
    QtVariantProperty *sub_item;

    QObject::connect(manager_, SIGNAL(valueChanged(QtProperty *, const QVariant &)),
              this, SLOT(onValueChanged(QtProperty *, const QVariant &)));

    top_item_ = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), QTranslator::tr("Origin"));

    // Create position properties
    item = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), QTranslator::tr("Position"));
    sub_item = manager_->addProperty(QtVariantPropertyManager::enumTypeId(), QTranslator::tr("Position Units"));
    sub_item->setAttribute(Common::attributeStr(EnumNames), QStringList() << QTranslator::tr("in") << QTranslator::tr("m")) ;
    item->addSubProperty(sub_item);
    sub_item = manager_->addProperty(QVariant::Double, QTranslator::tr("X"));
    sub_item->setAttribute(Common::attributeStr(Decimals), 6);
    item->addSubProperty(sub_item);
    sub_item = manager_->addProperty(QVariant::Double, QTranslator::tr("Y"));
    sub_item->setAttribute(Common::attributeStr(Decimals), 6);
    item->addSubProperty(sub_item);
    sub_item = manager_->addProperty(QVariant::Double, QTranslator::tr("Z"));
    sub_item->setAttribute(Common::attributeStr(Decimals), 6);
    item->addSubProperty(sub_item);
    top_item_->addSubProperty(item);

    // Create orientation properties
    item = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), QTranslator::tr("Orientation"));
    sub_item = manager_->addProperty(QtVariantPropertyManager::enumTypeId(), QTranslator::tr("Orientation Units"));
    sub_item->setAttribute(Common::attributeStr(EnumNames), QStringList() << QTranslator::tr("deg") << QTranslator::tr("rad"));
    item->addSubProperty(sub_item);
    sub_item = manager_->addProperty(QVariant::Double, QTranslator::tr("Roll"));
    sub_item->setAttribute(Common::attributeStr(Decimals), 6);
    item->addSubProperty(sub_item);
    sub_item = manager_->addProperty(QVariant::Double, QTranslator::tr("Pitch"));
    sub_item->setAttribute(Common::attributeStr(Decimals), 6);
    item->addSubProperty(sub_item);
    sub_item = manager_->addProperty(QVariant::Double, QTranslator::tr("Yaw"));
    sub_item->setAttribute(Common::attributeStr(Decimals), 6);
    item->addSubProperty(sub_item);
    top_item_->addSubProperty(item);


    loading_ = false;
  }

  OriginProperty::~OriginProperty()
  {
    delete manager_;
    delete factory_;
  }

  void OriginProperty::loadData()
  {
    loading_ = true;
    QtVariantProperty *item;
    QString name;
    double r, p, y;
    double x_pos, y_pos, z_pos;
    QList<QtProperty *> sub_items;

    sub_items = top_item_->subProperties()[0]->subProperties();
    for (int i = 0; i < sub_items.length(); ++i)
    {
      item = static_cast<QtVariantProperty *>(sub_items[i]);
      name = item->propertyName();
      
      x_pos = origin_.position.x*(metric_units_[0]) + meterToInch(origin_.position.x)*(1.0-metric_units_[0]);
      y_pos = origin_.position.y*(metric_units_[0]) + meterToInch(origin_.position.y)*(1.0-metric_units_[0]);
      z_pos = origin_.position.z*(metric_units_[0]) + meterToInch(origin_.position.z)*(1.0-metric_units_[0]);

      if (name == "Position Units")
        item->setValue(metric_units_[0]);
      else if (name == "X")
        item->setValue(x_pos);
      else if (name == "Y")
        item->setValue(y_pos);
      else if (name == "Z")
        item->setValue(z_pos);
    }

    sub_items = top_item_->subProperties()[1]->subProperties();
    for (int i = 0; i < sub_items.length(); ++i)
    {
      item = static_cast<QtVariantProperty *>(sub_items[i]);
      name = item->propertyName();

      origin_.rotation.getRPY(r, p, y);
      r = r*(metric_units_[1]) + radianToDegree(r)*(1.0-metric_units_[1]);
      p = p*(metric_units_[1]) + radianToDegree(p)*(1.0-metric_units_[1]);
      y = y*(metric_units_[1]) + radianToDegree(y)*(1.0-metric_units_[1]);
      
      if (name == "Orientation Units")
        item->setValue(metric_units_[1]);
      else if (name == "Roll")
        item->setValue(r);
      else if (name == "Pitch")
        item->setValue(p);
      else if (name == "Yaw")
        item->setValue(y);
    }

    loading_ = false;
  }

  void OriginProperty::loadFactoryForManager(QtTreePropertyBrowserSharedPtr& property_editor)
  {
    property_editor->setFactoryForManager(manager_, factory_);
  }

  void OriginProperty::onValueChanged(QtProperty *property, const QVariant &val)
  {
    if (loading_)
      return;

    double r, p, y;
    QString name = property->propertyName();
    if (name == "Position Units") 
    {
      metric_units_[0] = val.toDouble();
      loadData();
    }
    else if (name == "X")
      origin_.position.x = val.toDouble()*(metric_units_[0]) + inchToMeter(val.toDouble())*(1.0-metric_units_[0]);
    else if (name == "Y")
      origin_.position.y = val.toDouble()*(metric_units_[0]) + inchToMeter(val.toDouble())*(1.0-metric_units_[0]);
    else if (name == "Z")
      origin_.position.z = val.toDouble()*(metric_units_[0]) + inchToMeter(val.toDouble())*(1.0-metric_units_[0]);
    else if (name == "Orientation Units") 
    {
      metric_units_[1] = val.toDouble();
      loadData();
    }
    else if (name == "Roll" || name == "Pitch" || name == "Yaw")
    {
      origin_.rotation.getRPY(r, p, y);

      if (name == "Roll")
        r = val.toDouble()*(metric_units_[1]) + degreeToRadian(val.toDouble())*(1.0-metric_units_[1]);
      else if (name == "Pitch")
        p = val.toDouble()*(metric_units_[1]) + degreeToRadian(val.toDouble())*(1.0-metric_units_[1]);
      else if (name == "Yaw")
        y = val.toDouble()*(metric_units_[1]) + degreeToRadian(val.toDouble())*(1.0-metric_units_[1]);

      origin_.rotation.setFromRPY(r, p, y);
    }

    emit OriginProperty::valueChanged(property, val);
  }
  
  double OriginProperty::meterToInch(double val)
  {
    return val/0.0254;
  }

  double OriginProperty::inchToMeter(double val)
  {
    return val*0.0254;
  }

  double OriginProperty::radianToDegree(double val)
  {
    return val*(180/M_PI);
  }

  double OriginProperty::degreeToRadian(double val)
  {
    return val*(M_PI/180);
  }
}

