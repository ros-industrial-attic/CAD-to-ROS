#include "include/urdf_editor/joint_tree_property.h"

namespace urdf_editor
{
  JointTreeProperty::JointTreeProperty():manager_(new QtVariantPropertyManager())
  {
    QtVariantProperty *item;
    QtVariantProperty *sub_item;

//    QObject::connect(manager_, SIGNAL(valueChanged(QtProperty *, const QVariant &)),
//              this, SLOT(jointValueChanged(QtProperty *, const QVariant &)));

    top_item_ = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), tr("Joint Properties"));

    item = manager_->addProperty(QVariant::String, tr("Name"));
    top_item_->addSubProperty(item);

    item = manager_->addProperty(QtVariantPropertyManager::enumTypeId(), tr("Type"));
    item->setAttribute(Common::attributeStr(EnumNames), QStringList() << "Revolute" << "Continuous" << "Prismatic" << "Fixed" << "Floating" << "Planar");
    top_item_->addSubProperty(item);

    item = manager_->addProperty(QVariant::String, tr("Parent"));
    item->setAttribute(Common::attributeStr(ReadOnly), true);
    top_item_->addSubProperty(item);

    item = manager_->addProperty(QtVariantPropertyManager::enumTypeId(), tr("Child"));
    top_item_->addSubProperty(item);

    Common::addOriginProperty(manager_, top_item_);

    item = manager_->addProperty(QVariant::Vector3D, tr("Axis"));
    item->setAttribute(Common::attributeStr(Decimals), 6);
    top_item_->addSubProperty(item);

    item = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), tr("Calibration"));
    sub_item = manager_->addProperty(QVariant::Double, tr("Rising"));
    sub_item->setAttribute(Common::attributeStr(Minimum), 0.0);
    sub_item->setAttribute(Common::attributeStr(Decimals), 6);
    item->addSubProperty(sub_item);
    sub_item = manager_->addProperty(QVariant::Double, tr("Falling"));
    sub_item->setAttribute(Common::attributeStr(Minimum), 0.0);
    sub_item->setAttribute(Common::attributeStr(Decimals), 6);
    item->addSubProperty(sub_item);
    top_item_->addSubProperty(item);

    item = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), tr("Dynamics"));
    sub_item = manager_->addProperty(QVariant::Double, tr("Damping"));
    sub_item->setAttribute(Common::attributeStr(Minimum), 0.0);
    sub_item->setAttribute(Common::attributeStr(Decimals), 6);
    item->addSubProperty(sub_item);
    sub_item = manager_->addProperty(QVariant::Double, tr("Friction"));
    sub_item->setAttribute(Common::attributeStr(Minimum), 0.0);
    sub_item->setAttribute(Common::attributeStr(Decimals), 6);
    item->addSubProperty(sub_item);
    top_item_->addSubProperty(item);

    item = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), tr("Limit"));
    sub_item = manager_->addProperty(QVariant::Double, tr("Lower"));
    sub_item->setAttribute(Common::attributeStr(Decimals), 6);
    item->addSubProperty(sub_item);
    sub_item = manager_->addProperty(QVariant::Double, tr("Upper"));
    sub_item->setAttribute(Common::attributeStr(Decimals), 6);
    item->addSubProperty(sub_item);
    sub_item = manager_->addProperty(QVariant::Double, tr("Effort"));
    sub_item->setAttribute(Common::attributeStr(Minimum), 0.0);
    sub_item->setAttribute(Common::attributeStr(Decimals), 6);
    sub_item = manager_->addProperty(QVariant::Double, tr("Velocity"));
    sub_item->setAttribute(Common::attributeStr(Minimum), 0.0);
    sub_item->setAttribute(Common::attributeStr(Decimals), 6);
    item->addSubProperty(sub_item);
    top_item_->addSubProperty(item);

    item = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), tr("Mimic"));
    sub_item = manager_->addProperty(QtVariantPropertyManager::enumTypeId(), tr("Joint"));
    sub_item->setAttribute(Common::attributeStr(EnumNames), QStringList() << "joint list");
    item->addSubProperty(sub_item);
    sub_item = manager_->addProperty(QVariant::Double, tr("Multiplier"));
    sub_item->setAttribute(Common::attributeStr(Decimals), 6);
    item->addSubProperty(sub_item);
    sub_item = manager_->addProperty(QVariant::Double, tr("Offset"));
    sub_item->setAttribute(Common::attributeStr(Decimals), 6);
    item->addSubProperty(sub_item);
    top_item_->addSubProperty(item);

    item = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), tr("Safety Controller"));
    sub_item = manager_->addProperty(QVariant::Double, tr("Soft Lower Limit"));
    sub_item->setAttribute(Common::attributeStr(Decimals), 6);
    item->addSubProperty(sub_item);
    sub_item = manager_->addProperty(QVariant::Double, tr("Soft Upper Limit"));
    sub_item->setAttribute(Common::attributeStr(Decimals), 6);
    item->addSubProperty(sub_item);
    sub_item = manager_->addProperty(QVariant::Double, tr("K Position"));
    sub_item->setAttribute(Common::attributeStr(Decimals), 6);
    item->addSubProperty(sub_item);
    sub_item = manager_->addProperty(QVariant::Double, tr("K Velocity"));
    sub_item->setAttribute(Common::attributeStr(Decimals), 6);
    item->addSubProperty(sub_item);
    top_item_->addSubProperty(item);
  }

  JointTreeProperty::~JointTreeProperty()
  {
    delete manager_;
    delete top_item_;
  }

  void JointTreeProperty::jointValueChanged(QtProperty *property, const QVariant &val)
  {
    qDebug() << "test";
  }
}
