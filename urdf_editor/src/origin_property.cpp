
#include <urdf_editor/origin_property.h>
#include <urdf_editor/common.h>


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
    item = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), QTranslator::tr("Position (m)"));
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
    item = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), QTranslator::tr("Orientation (rad)"));
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
    QList<QtProperty *> sub_items;

    sub_items = top_item_->subProperties()[0]->subProperties();
    for (int i = 0; i < sub_items.length(); ++i)
    {
      item = static_cast<QtVariantProperty *>(sub_items[i]);
      name = item->propertyName();
      if (name == "X")
        item->setValue(origin_.position.x);
      else if (name == "Y")
        item->setValue(origin_.position.y);
      else if (name == "Z")
        item->setValue(origin_.position.z);
    }

    sub_items = top_item_->subProperties()[1]->subProperties();
    for (int i = 0; i < sub_items.length(); ++i)
    {
      item = static_cast<QtVariantProperty *>(sub_items[i]);
      name = item->propertyName();

      origin_.rotation.getRPY(r, p, y);

      if (name == "Roll")
        item->setValue(r);
      else if (name == "Pitch")
        item->setValue(p);
      else if (name == "Yaw")
        item->setValue(y);
    }

    loading_ = false;
  }

  void OriginProperty::loadFactoryForManager(boost::shared_ptr<QtTreePropertyBrowser> &property_editor)
  {
    property_editor->setFactoryForManager(manager_, factory_);
  }

  void OriginProperty::onValueChanged(QtProperty *property, const QVariant &val)
  {
    if (loading_)
      return;

    double r, p, y;
    QString name = property->propertyName();

    if (name == "X")
      origin_.position.x = val.toDouble();
    else if (name == "Y")
      origin_.position.y = val.toDouble();
    else if (name == "Z")
      origin_.position.z = val.toDouble();
    else if (name == "Roll" || name == "Pitch" || name == "Yaw")
    {
      origin_.rotation.getRPY(r, p, y);

      if (name == "Roll")
        r = val.toDouble();
      else if (name == "Pitch")
        p = val.toDouble();
      else if (name == "Yaw")
        y = val.toDouble();

      origin_.rotation.setFromRPY(r, p, y);
    }

    emit OriginProperty::valueChanged(property, val);
  }
}
