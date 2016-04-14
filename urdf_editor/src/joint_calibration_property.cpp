
#include <urdf_editor/joint_calibration_property.h>
#include <urdf_editor/common.h>


namespace urdf_editor
{
  JointCalibrationProperty::JointCalibrationProperty(boost::shared_ptr<urdf::JointCalibration> calibration): calibration_(calibration), manager_(new QtVariantPropertyManager()), factory_(new QtVariantEditorFactory())
  {
    loading_ = true;
    QtVariantProperty *item;

    QObject::connect(manager_, SIGNAL(valueChanged(QtProperty *, const QVariant &)),
              this, SLOT(onValueChanged(QtProperty *, const QVariant &)));

    top_item_ = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), tr("Calibration"));
    item = manager_->addProperty(QVariant::Double, tr("Rising"));
    item->setAttribute(Common::attributeStr(Minimum), 0.0);
    item->setAttribute(Common::attributeStr(Decimals), 6);
    top_item_->addSubProperty(item);
    item = manager_->addProperty(QVariant::Double, tr("Falling"));
    item->setAttribute(Common::attributeStr(Minimum), 0.0);
    item->setAttribute(Common::attributeStr(Decimals), 6);
    top_item_->addSubProperty(item);

    loading_ = false;
  }

  JointCalibrationProperty::~JointCalibrationProperty()
  {
    delete manager_;
    delete factory_;
  }

  void JointCalibrationProperty::loadData()
  {
    loading_ = true;
    QtVariantProperty *item;
    QString name;
    QList<QtProperty *> sub_items = top_item_->subProperties();
    for (int i = 0; i < sub_items.length(); ++i)
    {
      item = static_cast<QtVariantProperty *>(sub_items[i]);
      name = item->propertyName();
      if (name == "Rising")
      {
        if (!calibration_->rising)
          calibration_->rising.reset(new double);
        item->setValue(*calibration_->rising);
      }
      else if (name == "Falling")
      {
        if (!calibration_->falling)
          calibration_->falling.reset(new double);
        item->setValue(*calibration_->falling);
      }
    }
    loading_ = false;
  }

  void JointCalibrationProperty::loadFactoryForManager(boost::shared_ptr<QtTreePropertyBrowser> &property_editor)
  {
    property_editor->setFactoryForManager(manager_, factory_);
  }

  void JointCalibrationProperty::onValueChanged(QtProperty *property, const QVariant &val)
  {
    if (loading_)
      return;

    QString name = property->propertyName();
    if (name == "Rising")
      *calibration_->rising = val.toDouble();
    else if (name == "Falling")
      *calibration_->falling = val.toDouble();

    emit JointCalibrationProperty::valueChanged(property, val);
  }
}
