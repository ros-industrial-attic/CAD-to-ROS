#include "include/urdf_editor/link_tree_property.h"

namespace urdf_editor
{
  LinkTreeProperty::LinkTreeProperty():manager_(new QtVariantPropertyManager())
  {
    QObject::connect(manager_, SIGNAL(valueChanged(QtProperty *, const QVariant &)),
              this, SLOT(linkValueChanged(QtProperty *, const QVariant &)));
    top_item_ = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), tr("Link Properties"));

    QtVariantProperty *item = manager_->addProperty(QVariant::String, tr("Name"));
    top_item_->addSubProperty(item);

    addVisualProperty();
    addCollisionProperty();
    addInertialProperty();
  }

  LinkTreeProperty::~LinkTreeProperty()
  {
    delete manager_;
    delete top_item_;
  }

  void LinkTreeProperty::addVisualProperty()
  {
    QtVariantProperty *item;
    QtVariantProperty *visual;
    QtVariantProperty *material;

    visual = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), tr("Visual (optional)"));
    item = manager_->addProperty(QVariant::String, tr("Name"));
    visual->addSubProperty(item);

    Common::addOriginProperty(manager_, visual);

    material = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), tr("Material (optional)"));

    item = manager_->addProperty(QVariant::Color, tr("Color"));
    material->addSubProperty(item);

    item = manager_->addProperty(QVariant::String, tr("Texture"));
    material->addSubProperty(item);

    visual->addSubProperty(material);

    top_item_->addSubProperty(visual);
  }

  void LinkTreeProperty::addCollisionProperty()
  {
    QtVariantProperty *item;
    QtVariantProperty *collision;

    collision = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), tr("Collision (optional)"));
    item = manager_->addProperty(QVariant::String, tr("Name"));
    collision->addSubProperty(item);

    Common::addOriginProperty(manager_, collision);
    top_item_->addSubProperty(collision);
  }

  void LinkTreeProperty::addInertialProperty()
  {
    QtVariantProperty *item;
    QtVariantProperty *sub_item;
    QtVariantProperty *inertial;

    inertial = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), tr("Inertial (optional)"));
    item = manager_->addProperty(QVariant::Double, tr("Mass (kg)"));
    item->setAttribute(Common::attributeStr(Minimum), 0.0);
    inertial->addSubProperty(item);

    item = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), tr("Inertia"));
    sub_item = manager_->addProperty(QVariant::Double, tr("Ixx"));
    item->addSubProperty(sub_item);
    sub_item = manager_->addProperty(QVariant::Double, tr("Iyy"));
    item->addSubProperty(sub_item);
    sub_item = manager_->addProperty(QVariant::Double, tr("Izz"));
    item->addSubProperty(sub_item);
    sub_item = manager_->addProperty(QVariant::Double, tr("Ixy"));
    item->addSubProperty(sub_item);
    sub_item = manager_->addProperty(QVariant::Double, tr("Ixz"));
    item->addSubProperty(sub_item);
    sub_item = manager_->addProperty(QVariant::Double, tr("Iyz"));
    item->addSubProperty(sub_item);
    inertial->addSubProperty(item);

    Common::addOriginProperty(manager_, inertial);

    top_item_->addSubProperty(inertial);

  }

  void LinkTreeProperty::linkValueChanged(QtProperty *property, const QVariant &val)
  {
    qDebug() << "test";
  }
}

