
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
    QList<QtProperty *> sub_items = top_item_->subProperties();
    for (int i = 0; i < sub_items.length(); ++i)
    {
      item = static_cast<QtVariantProperty *>(sub_items[i]);
      name = item->propertyName();
      if (name == "X")
        item->setValue(axis_.x);
      else if (name == "Y")
        item->setValue(axis_.y);
      else if (name == "Z")
        item->setValue(axis_.z);
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
    if (name == "X")
      axis_.x = val.toDouble();
    else if (name == "Y")
      axis_.y = val.toDouble();
    else if (name == "Z")
      axis_.z = val.toDouble();

    emit JointAxisProperty::valueChanged(property, val);
  }
}
