#include "include/urdf_editor/link_property.h"

namespace urdf_editor
{
  // Link Inertial Property
  LinkInertialProperty::LinkInertialProperty(boost::shared_ptr<urdf::Inertial> inertial): inertial_(inertial), manager_(new QtVariantPropertyManager()), factory_(new QtVariantEditorFactory())
  {
    QtVariantProperty *item;
    QtVariantProperty *sub_item;
    double p_norm, r_norm;
    urdf::Pose origin;

    QObject::connect(manager_, SIGNAL(valueChanged(QtProperty *, const QVariant &)),
              this, SLOT(linkInertialValueChanged(QtProperty *, const QVariant &)));

    top_item_ = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), tr("Inertial"));

    item = manager_->addProperty(QVariant::Double, tr("Mass (kg)"));
    item->setAttribute(Common::attributeStr(Minimum), 0.0);
    top_item_->addSubProperty(item);

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
    top_item_->addSubProperty(item);

    origin = inertial_->origin;
    p_norm = origin.position.x * origin.position.x;
    p_norm += origin.position.y * origin.position.y;
    p_norm += origin.position.z * origin.position.z;
    r_norm = origin.rotation.x * origin.rotation.x;
    r_norm += origin.rotation.y * origin.rotation.y;
    r_norm += origin.rotation.z * origin.rotation.z;
    if (p_norm > 0.0 || r_norm > 0.0)
    {
      origin_property_.reset(new OriginProperty(inertial_->origin));
      top_item_->addSubProperty(origin_property_->getTopItem());
    }
  }

  LinkInertialProperty::~LinkInertialProperty()
  {
    delete manager_;
    delete factory_;
  }

  void LinkInertialProperty::loadFactoryForManager(boost::shared_ptr<QtTreePropertyBrowser> property_editor)
  {
    property_editor->setFactoryForManager(manager_, factory_);
    if (origin_property_)
      origin_property_->loadFactoryForManager(property_editor);
  }

  void LinkInertialProperty::linkInertialValueChanged(QtProperty *property, const QVariant &val)
  {
    QString name = property->propertyName();
    if (name == "Mass (kg)")
    {
      inertial_->mass = val.toDouble();
    }
    else if (name == "Ixx")
    {
      inertial_->ixx = val.toDouble();
    }
    else if (name == "Iyy")
    {
      inertial_->iyy = val.toDouble();
    }
    else if (name == "Izz")
    {
      inertial_->izz = val.toDouble();
    }
    else if (name == "Ixy")
    {
      inertial_->ixy = val.toDouble();
    }
    else if (name == "Ixz")
    {
      inertial_->ixz = val.toDouble();
    }
    else if (name == "Iyz")
    {
      inertial_->iyz = val.toDouble();
    }
  }

  // Link Property
  LinkProperty::LinkProperty(boost::shared_ptr<urdf::Link> link):link_(link), manager_(new QtVariantPropertyManager()), factory_(new QtVariantEditorFactory())
  {


    QObject::connect(manager_, SIGNAL(valueChanged(QtProperty *, const QVariant &)),
              this, SLOT(linkValueChanged(QtProperty *, const QVariant &)));

    name_item_ = manager_->addProperty(QVariant::String, tr("Name"));
    name_item_->setValue(QString::fromStdString(link_->name));

    if (link_->inertial)
      inertial_property_.reset(new LinkInertialProperty(link_->inertial));

    //addVisualProperty();
    //addCollisionProperty();
    //addInertialProperty();
  }

  LinkProperty::~LinkProperty()
  {
    delete manager_;
    delete factory_;
  }

  void LinkProperty::loadProperty(boost::shared_ptr<QtTreePropertyBrowser> property_editor)
  {
    property_editor->clear();
    property_editor->setFactoryForManager(manager_, factory_);
    property_editor->addProperty(name_item_);

    if (inertial_property_)
    {
      inertial_property_->loadFactoryForManager(property_editor);
      property_editor->addProperty(inertial_property_->getTopItem());
    }
  }

  void LinkProperty::addVisualProperty()
  {
    QtVariantProperty *item;
    QtVariantProperty *visual;
    QtVariantProperty *material;

    visual = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), tr("Visual (optional)"));
    item = manager_->addProperty(QVariant::String, tr("Name"));
    visual->addSubProperty(item);

    Common::addOriginProperty(manager_, visual, link_->visual->origin);

    material = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), tr("Material (optional)"));

    item = manager_->addProperty(QVariant::Color, tr("Color"));
    material->addSubProperty(item);

    item = manager_->addProperty(QVariant::String, tr("Texture"));
    material->addSubProperty(item);

    visual->addSubProperty(material);

    top_item_->addSubProperty(visual);
  }

  void LinkProperty::addCollisionProperty()
  {
    QtVariantProperty *item;
    QtVariantProperty *collision;

    collision = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), tr("Collision (optional)"));
    item = manager_->addProperty(QVariant::String, tr("Name"));
    collision->addSubProperty(item);

    Common::addOriginProperty(manager_, collision, link_->collision->origin);

    top_item_->addSubProperty(collision);
  }

  void LinkProperty::addInertialProperty()
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

    Common::addOriginProperty(manager_, inertial, link_->inertial->origin);

    top_item_->addSubProperty(inertial);

  }

  void LinkProperty::linkValueChanged(QtProperty *property, const QVariant &val)
  {
    qDebug() << "test";
  }
}

