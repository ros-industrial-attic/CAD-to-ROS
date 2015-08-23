#include "include/urdf_editor/link_property.h"

namespace urdf_editor
{
  // Link Geometry Property
  LinkGeometryProperty::LinkGeometryProperty(boost::shared_ptr<urdf::Geometry> geometry): geometry_(geometry), manager_(new QtVariantPropertyManager()), factory_(new QtVariantEditorFactory())
  {
    QtVariantProperty *item;
    QtVariantProperty *sub_item;

    QObject::connect(manager_, SIGNAL(valueChanged(QtProperty *, const QVariant &)),
              this, SLOT(linkGeometryValueChanged(QtProperty *, const QVariant &)));
    //{SPHERE, BOX, CYLINDER, MESH}
    top_item_ = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), tr("Geometry"));
    item = manager_->addProperty(QtVariantPropertyManager::enumTypeId(), tr("Type"));
    item->setAttribute(Common::attributeStr(EnumNames), QStringList() << "SPHERE" << "BOX" << "CYLINDER" << "MESH");
    item->setValue(geometry_->type);
    top_item_->addSubProperty(item);

    if (geometry_->type == urdf::Geometry::BOX)
    {
      boost::shared_ptr<urdf::Box> box = boost::static_pointer_cast<urdf::Box>(geometry_);
      item = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), tr("Size"));
      sub_item = manager_->addProperty(QVariant::Double, tr("Length X (m)"));
      sub_item->setValue(box->dim.x);
      item->addSubProperty(sub_item);

      sub_item = manager_->addProperty(QVariant::Double, tr("Length Y (m)"));
      sub_item->setValue(box->dim.y);
      item->addSubProperty(sub_item);

      sub_item = manager_->addProperty(QVariant::Double, tr("Length Z (m)"));
      sub_item->setValue(box->dim.z);
      item->addSubProperty(sub_item);
      top_item_->addSubProperty(item);
    }
    else if (geometry_->type == urdf::Geometry::CYLINDER)
    {
      boost::shared_ptr<urdf::Cylinder> cylinder = boost::static_pointer_cast<urdf::Cylinder>(geometry_);
      item = manager_->addProperty(QVariant::Double, tr("Radius (m)"));
      item->setValue(cylinder->radius);
      top_item_->addSubProperty(item);

      item = manager_->addProperty(QVariant::Double, tr("Length (m)"));
      item->setValue(cylinder->length);
      top_item_->addSubProperty(item);
    }
    else if (geometry_->type == urdf::Geometry::SPHERE)
    {
      boost::shared_ptr<urdf::Sphere> sphere = boost::static_pointer_cast<urdf::Sphere>(geometry_);
      item = manager_->addProperty(QVariant::Double, tr("Radius (m)"));
      item->setValue(sphere->radius);
      top_item_->addSubProperty(item);
    }
    else if (geometry_->type == urdf::Geometry::MESH)
    {
      boost::shared_ptr<urdf::Mesh> mesh = boost::static_pointer_cast<urdf::Mesh>(geometry_);
      item = manager_->addProperty(QVariant::String, tr("File Name"));
      item->setValue(QString::fromStdString(mesh->filename));
      top_item_->addSubProperty(item);

      item = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), tr("Scale"));
      sub_item = manager_->addProperty(QVariant::Double, tr("X"));
      sub_item->setValue(mesh->scale.x);
      item->addSubProperty(sub_item);
      sub_item = manager_->addProperty(QVariant::Double, tr("Y"));
      sub_item->setValue(mesh->scale.y);
      item->addSubProperty(sub_item);
      sub_item = manager_->addProperty(QVariant::Double, tr("Z"));
      sub_item->setValue(mesh->scale.z);
      item->addSubProperty(sub_item);
      top_item_->addSubProperty(item);
    }

  }

  LinkGeometryProperty::~LinkGeometryProperty()
  {
    delete manager_;
    delete factory_;
  }

  void LinkGeometryProperty::loadFactoryForManager(boost::shared_ptr<QtTreePropertyBrowser> property_editor)
  {
    property_editor->setFactoryForManager(manager_, factory_);
  }

  void LinkGeometryProperty::linkGeometryValueChanged(QtProperty *property, const QVariant &val)
  {
    QString name = property->propertyName();
    if (geometry_->type == urdf::Geometry::BOX)
    {
      boost::shared_ptr<urdf::Box> box = boost::static_pointer_cast<urdf::Box>(geometry_);
      if (name == "Length X (m)")
      {
        box->dim.x = val.toDouble();
      }
      else if (name == "Length Y (m)")
      {
        box->dim.y = val.toDouble();
      }
      else if (name == "Length Z (m)")
      {
        box->dim.z = val.toDouble();
      }
    }
    else if (geometry_->type == urdf::Geometry::CYLINDER)
    {
      boost::shared_ptr<urdf::Cylinder> cylinder = boost::static_pointer_cast<urdf::Cylinder>(geometry_);
      if (name == "Radius (m)")
      {
        cylinder->radius = val.toDouble();
      }
      else if (name == "Length (m)")
      {
        cylinder->length = val.toDouble();
      }
    }
    else if (geometry_->type == urdf::Geometry::SPHERE)
    {
      boost::shared_ptr<urdf::Sphere> sphere = boost::static_pointer_cast<urdf::Sphere>(geometry_);
      if (name == "Radius (m)")
      {
        sphere->radius = val.toDouble();
      }
    }
    else if (geometry_->type == urdf::Geometry::MESH)
    {
      boost::shared_ptr<urdf::Mesh> mesh = boost::static_pointer_cast<urdf::Mesh>(geometry_);
      if (name == "File Name")
      {
        mesh->filename = val.toString().toStdString();
      }
      else if (name == "X")
      {
        mesh->scale.x = val.toDouble();
      }
      else if (name == "Y")
      {
        mesh->scale.y = val.toDouble();
      }
      else if (name == "Z")
      {
        mesh->scale.z = val.toDouble();
      }
    }
  }

  // Link Collision Property
  LinkCollisionProperty::LinkCollisionProperty(boost::shared_ptr<urdf::Collision> collision): collision_(collision), manager_(new QtVariantPropertyManager()), factory_(new QtVariantEditorFactory())
  {
    QtVariantProperty *item;
    double p_norm, r_norm;
    urdf::Pose origin;

    QObject::connect(manager_, SIGNAL(valueChanged(QtProperty *, const QVariant &)),
              this, SLOT(linkCollisionValueChanged(QtProperty *, const QVariant &)));

    top_item_ = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), tr("Collision"));
    item = manager_->addProperty(QVariant::String, tr("Name"));
    item->setValue(QString::fromStdString(collision_->group_name));
    top_item_->addSubProperty(item);

    origin = collision_->origin;
    p_norm = origin.position.x * origin.position.x;
    p_norm += origin.position.y * origin.position.y;
    p_norm += origin.position.z * origin.position.z;
    r_norm = origin.rotation.x * origin.rotation.x;
    r_norm += origin.rotation.y * origin.rotation.y;
    r_norm += origin.rotation.z * origin.rotation.z;
    if (p_norm > 0.0 || r_norm > 0.0)
    {
      origin_property_.reset(new OriginProperty(collision_->origin));
      top_item_->addSubProperty(origin_property_->getTopItem());
    }

    if (collision_->geometry)
    {
      geometry_property_.reset(new LinkGeometryProperty(collision_->geometry));
      top_item_->addSubProperty(geometry_property_->getTopItem());
    }

  }

  LinkCollisionProperty::~LinkCollisionProperty()
  {
    delete manager_;
    delete factory_;
  }

  void LinkCollisionProperty::loadFactoryForManager(boost::shared_ptr<QtTreePropertyBrowser> property_editor)
  {
    property_editor->setFactoryForManager(manager_, factory_);
    if (origin_property_)
      origin_property_->loadFactoryForManager(property_editor);

    if (geometry_property_)
      geometry_property_->loadFactoryForManager(property_editor);

  }

  void LinkCollisionProperty::linkCollisionValueChanged(QtProperty *property, const QVariant &val)
  {
    QString name = property->propertyName();
    if (name == "Name")
    {
      collision_->group_name = val.toString().toStdString();
    }

  }

  // Link Material Property
  LinkNewMaterialProperty::LinkNewMaterialProperty(boost::shared_ptr<urdf::Material> material): material_(material), manager_(new QtVariantPropertyManager()), factory_(new QtVariantEditorFactory())
  {
    QtVariantProperty *item;

    QObject::connect(manager_, SIGNAL(valueChanged(QtProperty *, const QVariant &)),
              this, SLOT(linkNewMaterialValueChanged(QtProperty *, const QVariant &)));

    top_item_ = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), tr("Material"));

    item = manager_->addProperty(QVariant::String, tr("Name"));
    item->setValue(QString::fromStdString(material_->name));
    top_item_->addSubProperty(item);

    item = manager_->addProperty(QVariant::Color, tr("Color"));
    item->setValue(QColor::fromRgba(qRgba(material_->color.r * 255.0, material_->color.g * 255.0, material_->color.b * 255.0, material_->color.a * 255.0)));
    top_item_->addSubProperty(item);

    item = manager_->addProperty(QVariant::String, tr("Texture"));
    item->setValue(QString::fromStdString(material_->texture_filename));
    top_item_->addSubProperty(item);

  }

  LinkNewMaterialProperty::~LinkNewMaterialProperty()
  {
    delete manager_;
    delete factory_;
  }

  void LinkNewMaterialProperty::loadFactoryForManager(boost::shared_ptr<QtTreePropertyBrowser> property_editor)
  {
    property_editor->setFactoryForManager(manager_, factory_);
  }

  void LinkNewMaterialProperty::linkNewMaterialValueChanged(QtProperty *property, const QVariant &val)
  {
    QString name = property->propertyName();
    if (name == "Name")
    {
      material_->name = val.toString().toStdString();
    }
    else if (name == "Red")
    {
      material_->color.r = val.toDouble()/255.0;
    }
    else if (name == "Green")
    {
      material_->color.g = val.toDouble()/255.0;
    }
    else if (name == "Blue")
    {
      material_->color.b = val.toDouble()/255.0;
    }
    else if (name == "Alpha")
    {
      material_->color.a = val.toDouble()/255.0;
    }
    else if (name == "Texture")
    {
      material_->texture_filename = val.toString().toStdString();
    }
  }

  // Link Visual Property
  LinkVisualProperty::LinkVisualProperty(boost::shared_ptr<urdf::Visual> visual): visual_(visual), manager_(new QtVariantPropertyManager()), factory_(new QtVariantEditorFactory())
  {
    QtVariantProperty *item;
    double p_norm, r_norm;
    urdf::Pose origin;

    QObject::connect(manager_, SIGNAL(valueChanged(QtProperty *, const QVariaboxnt &)),
              this, SLOT(linkVisualValueChanged(QtProperty *, const QVariant &)));

    top_item_ = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), tr("Visual"));
    item = manager_->addProperty(QVariant::String, tr("Name"));
    item->setValue(QString::fromStdString(visual_->group_name));
    top_item_->addSubProperty(item);

    origin = visual_->origin;
    p_norm = origin.position.x * origin.position.x;
    p_norm += origin.position.y * origin.position.y;
    p_norm += origin.position.z * origin.position.z;
    r_norm = origin.rotation.x * origin.rotation.x;
    r_norm += origin.rotation.y * origin.rotation.y;
    r_norm += origin.rotation.z * origin.rotation.z;
    if (p_norm > 0.0 || r_norm > 0.0)
    {
      origin_property_.reset(new OriginProperty(visual_->origin));
      top_item_->addSubProperty(origin_property_->getTopItem());
    }

    if (visual_->material)
    {
      new_material_property_.reset(new LinkNewMaterialProperty(visual_->material));
      top_item_->addSubProperty(new_material_property_->getTopItem());
    }

    if (visual_->geometry)
    {
      geometry_property_.reset(new LinkGeometryProperty(visual_->geometry));
      top_item_->addSubProperty(geometry_property_->getTopItem());
    }
    else
    {
      //need to create one since it is not optional
    }

  }

  LinkVisualProperty::~LinkVisualProperty()
  {
    delete manager_;
    delete factory_;
  }

  void LinkVisualProperty::loadFactoryForManager(boost::shared_ptr<QtTreePropertyBrowser> property_editor)
  {
    property_editor->setFactoryForManager(manager_, factory_);
    if (origin_property_)
      origin_property_->loadFactoryForManager(property_editor);

    if (new_material_property_)
      new_material_property_->loadFactoryForManager(property_editor);

    if (geometry_property_)
      geometry_property_->loadFactoryForManager(property_editor);

  }

  void LinkVisualProperty::linkVisualValueChanged(QtProperty *property, const QVariant &val)
  {
    QString name = property->propertyName();
    if (name == "Name")
    {
      visual_->group_name = val.toString().toStdString();
    }

  }

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

    //top_item_ = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), tr("Link Properties"));

    name_item_ = manager_->addProperty(QVariant::String, tr("Name"));
    name_item_->setValue(QString::fromStdString(link_->name));

    if (link_->inertial)
      inertial_property_.reset(new LinkInertialProperty(link_->inertial));

    if (link_->visual)
      visual_property_.reset(new LinkVisualProperty(link_->visual));

    if (link_->collision)
      collision_property_.reset(new LinkCollisionProperty(link_->collision));

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

    if (visual_property_)
    {
      visual_property_->loadFactoryForManager(property_editor);
      property_editor->addProperty(visual_property_->getTopItem());
    }

    if (collision_property_)
    {
      collision_property_->loadFactoryForManager(property_editor);
      property_editor->addProperty(collision_property_->getTopItem());
    }
  }

  void LinkProperty::linkValueChanged(QtProperty *property, const QVariant &val)
  {
    QString name = property->propertyName();

    if (name == "Name")
    {
      link_->name = val.toString().toStdString();
    }
  }
}

