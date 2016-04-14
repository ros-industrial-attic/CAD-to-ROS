
#include <urdf_editor/joint_mimic_property.h>
#include <urdf_editor/common.h>


namespace urdf_editor
{
  JointMimicProperty::JointMimicProperty(boost::shared_ptr<urdf::JointMimic> mimic, QStringList &joint_names): mimic_(mimic), manager_(new QtVariantPropertyManager()), factory_(new QtVariantEditorFactory()), joint_names_(joint_names)
  {
    loading_ = true;
    QtVariantProperty *item;

    QObject::connect(manager_, SIGNAL(valueChanged(QtProperty *, const QVariant &)),
              this, SLOT(onValueChanged(QtProperty *, const QVariant &)));

    top_item_ = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), tr("Mimic"));
    item = manager_->addProperty(QtVariantPropertyManager::enumTypeId(), tr("Joint"));
    item->setAttribute(Common::attributeStr(EnumNames), joint_names_);
    top_item_->addSubProperty(item);
    item = manager_->addProperty(QVariant::Double, tr("Multiplier"));
    item->setAttribute(Common::attributeStr(Decimals), 6);
    top_item_->addSubProperty(item);
    item = manager_->addProperty(QVariant::Double, tr("Offset"));
    item->setAttribute(Common::attributeStr(Decimals), 6);
    top_item_->addSubProperty(item);

    loading_ = false;
  }

  JointMimicProperty::~JointMimicProperty()
  {
    delete manager_;
    delete factory_;
  }

  void JointMimicProperty::loadData()
  {
    loading_ = true;
    QtVariantProperty *item;
    QString name;
    QList<QtProperty *> sub_items = top_item_->subProperties();
    for (int i = 0; i < sub_items.length(); ++i)
    {
      item = static_cast<QtVariantProperty *>(sub_items[i]);
      name = item->propertyName();
      if (name == "Joint")
      {
        item->setAttribute(Common::attributeStr(EnumNames), joint_names_);
        item->setValue(joint_names_.indexOf(QString::fromStdString(mimic_->joint_name)));
      }
      else if (name == "Multiplier")
        item->setValue(mimic_->multiplier);
      else if (name == "Offset")
        item->setValue(mimic_->offset);
    }
    loading_ = false;
  }

  void JointMimicProperty::loadFactoryForManager(boost::shared_ptr<QtTreePropertyBrowser> &property_editor)
  {
    property_editor->setFactoryForManager(manager_, factory_);
  }

  void JointMimicProperty::onValueChanged(QtProperty *property, const QVariant &val)
  {
    if (loading_)
      return;

    QString name = property->propertyName();
    if (name == "Joint")
      mimic_->joint_name = val.toString().toStdString();
    else if (name == "Multiplier")
      mimic_->multiplier = val.toDouble();
    else if (name == "Offset")
      mimic_->offset = val.toDouble();

    emit JointMimicProperty::valueChanged(property, val);
  }
}
