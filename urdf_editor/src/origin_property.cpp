
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
    in_degrees_ = false;
    in_inches_ = false;


    QtVariantProperty *item;
    QtVariantProperty *sub_item;

    QObject::connect(manager_, SIGNAL(valueChanged(QtProperty *, const QVariant &)),
              this, SLOT(onValueChanged(QtProperty *, const QVariant &)));

    top_item_ = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), QTranslator::tr("Origin"));

    // Create position properties
    item = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), QTranslator::tr("Position"));
    sub_item = manager_->addProperty(QtVariantPropertyManager::enumTypeId(), QTranslator::tr("Position Units"));
    sub_item->setAttribute(Common::attributeStr(EnumNames), QStringList() << QTranslator::tr("m") << QTranslator::tr("in")) ;
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
    sub_item->setAttribute(Common::attributeStr(EnumNames), QStringList() << QTranslator::tr("rad") << QTranslator::tr("deg"));
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

    origin_.rotation.getRPY(r, p, y);
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
    double r_display, p_display, y_display;
    double x_pos, y_pos, z_pos;
    QList<QtProperty *> sub_items;


    sub_items = top_item_->subProperties()[0]->subProperties();
    for (int i = 0; i < sub_items.length(); ++i)
    {
      item = static_cast<QtVariantProperty *>(sub_items[i]);
      name = item->propertyName();
      
      x_pos = origin_.position.x*(1.0-in_inches_) + meterToInch(origin_.position.x)*(in_inches_);
      y_pos = origin_.position.y*(1.0-in_inches_) + meterToInch(origin_.position.y)*(in_inches_);
      z_pos = origin_.position.z*(1.0-in_inches_) + meterToInch(origin_.position.z)*(in_inches_);

      if (name == "Position Units")
        item->setValue(in_inches_);
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

      
      r_display = r*(1.0-in_degrees_) + radianToDegree(r)*(in_degrees_);
      p_display = p*(1.0-in_degrees_) + radianToDegree(p)*(in_degrees_);
      y_display = y*(1.0-in_degrees_) + radianToDegree(y)*(in_degrees_);
      
      if (name == "Orientation Units")
        item->setValue(in_degrees_);
      else if (name == "Roll")
        item->setValue(r_display);
      else if (name == "Pitch")
        item->setValue(p_display);
      else if (name == "Yaw")
        item->setValue(y_display);
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

    
    QString name = property->propertyName();
    if (name == "Position Units") 
    {
      in_inches_ = val.toDouble();
      loadData();
    }
    else if (name == "X")
      origin_.position.x = val.toDouble()*(1.0-in_inches_) + inchToMeter(val.toDouble())*(in_inches_);
    else if (name == "Y")
      origin_.position.y = val.toDouble()*(1.0-in_inches_) + inchToMeter(val.toDouble())*(in_inches_);
    else if (name == "Z")
      origin_.position.z = val.toDouble()*(1.0-in_inches_) + inchToMeter(val.toDouble())*(in_inches_);
    else if (name == "Orientation Units") 
    {
      in_degrees_ = val.toDouble();
      loadData();
    }
    else if (name == "Roll" || name == "Pitch" || name == "Yaw")
    {
      //origin_.rotation.getRPY(r, p, y);

      if (name == "Roll")
        r = val.toDouble()*(1.0-in_degrees_) + degreeToRadian(val.toDouble())*(in_degrees_);
      else if (name == "Pitch")
        p = val.toDouble()*(1.0-in_degrees_) + degreeToRadian(val.toDouble())*(in_degrees_);
      else if (name == "Yaw")
        y = val.toDouble()*(1.0-in_degrees_) + degreeToRadian(val.toDouble())*(in_degrees_);

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
    return val*(180/3.14159265);
  }

  double OriginProperty::degreeToRadian(double val)
  {
    return val*(3.14159265/180);
  }
}

