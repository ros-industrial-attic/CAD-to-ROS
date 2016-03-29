#include "urdf_editor/link_property.h"
#include <qt4/QtCore/qvariant.h>

namespace urdf_editor
{
  // Link Geometry Property
  LinkGeometryProperty::LinkGeometryProperty(boost::shared_ptr<urdf::Geometry> geometry): geometry_(geometry), manager_(new QtVariantPropertyManager()), factory_(new QtVariantEditorFactory())
  {
    loading_ = true;
    QtVariantProperty *item;
    QtVariantProperty *sub_item;

    QObject::connect(manager_, SIGNAL(valueChanged(QtProperty *, const QVariant &)),
              this, SLOT(onValueChanged(QtProperty *, const QVariant &)));
    //{SPHERE, BOX, CYLINDER, MESH}
    top_item_ = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), tr("Geometry"));
    item = manager_->addProperty(QtVariantPropertyManager::enumTypeId(), tr("Type"));
    item->setAttribute(Common::attributeStr(EnumNames), QStringList() << "SPHERE" << "BOX" << "CYLINDER" << "MESH");
    top_item_->addSubProperty(item);

    if (geometry_->type == urdf::Geometry::BOX)
    {
      boost::shared_ptr<urdf::Box> box = boost::static_pointer_cast<urdf::Box>(geometry_);
      item = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), tr("Size"));
      sub_item = manager_->addProperty(QVariant::Double, tr("Length X (m)"));
      item->addSubProperty(sub_item);

      sub_item = manager_->addProperty(QVariant::Double, tr("Length Y (m)"));
      item->addSubProperty(sub_item);

      sub_item = manager_->addProperty(QVariant::Double, tr("Length Z (m)"));
      item->addSubProperty(sub_item);
      top_item_->addSubProperty(item);
    }
    else if (geometry_->type == urdf::Geometry::CYLINDER)
    {
      boost::shared_ptr<urdf::Cylinder> cylinder = boost::static_pointer_cast<urdf::Cylinder>(geometry_);
      item = manager_->addProperty(QVariant::Double, tr("Radius (m)"));
      top_item_->addSubProperty(item);

      item = manager_->addProperty(QVariant::Double, tr("Length (m)"));
      top_item_->addSubProperty(item);
    }
    else if (geometry_->type == urdf::Geometry::SPHERE)
    {
      boost::shared_ptr<urdf::Sphere> sphere = boost::static_pointer_cast<urdf::Sphere>(geometry_);
      item = manager_->addProperty(QVariant::Double, tr("Radius (m)"));
      top_item_->addSubProperty(item);
    }
    else if (geometry_->type == urdf::Geometry::MESH)
    {
      boost::shared_ptr<urdf::Mesh> mesh = boost::static_pointer_cast<urdf::Mesh>(geometry_);
      item = manager_->addProperty(QVariant::String, tr("File Name"));
      top_item_->addSubProperty(item);

      item = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), tr("Scale"));
      sub_item = manager_->addProperty(QVariant::Double, tr("X"));
      item->addSubProperty(sub_item);
      sub_item = manager_->addProperty(QVariant::Double, tr("Y"));
      item->addSubProperty(sub_item);
      sub_item = manager_->addProperty(QVariant::Double, tr("Z"));
      item->addSubProperty(sub_item);
      top_item_->addSubProperty(item);
    }
    loading_ = false;
  }

  LinkGeometryProperty::~LinkGeometryProperty()
  {
    delete manager_;
    delete factory_;
  }

  void LinkGeometryProperty::loadData()
  {
    loading_ = true;
    QtVariantProperty *item;
    QString name;
    QList<QtProperty *> sub_items = top_item_->subProperties();
    for (int i = 0; i < sub_items.length(); ++i)
    {
      item = static_cast<QtVariantProperty *>(sub_items[i]);
      name = item->propertyName();

      if (name == "Type")
        item->setValue(geometry_->type);
      else
      {
        if (geometry_->type == urdf::Geometry::BOX)
        {
          boost::shared_ptr<urdf::Box> box = boost::static_pointer_cast<urdf::Box>(geometry_);
          if (name == "Length X (m)")
            item->setValue(box->dim.x);
          else if (name == "Length Y (m)")
            item->setValue(box->dim.y);
          else if (name == "Length Z (m)")
            item->setValue(box->dim.z);
        }
        else if (geometry_->type == urdf::Geometry::CYLINDER)
        {
          boost::shared_ptr<urdf::Cylinder> cylinder = boost::static_pointer_cast<urdf::Cylinder>(geometry_);
          if (name == "Radius (m)")
            item->setValue(cylinder->radius);
          else if (name == "Length (m)")
            item->setValue(cylinder->length);
        }
        else if (geometry_->type == urdf::Geometry::SPHERE)
        {
          boost::shared_ptr<urdf::Sphere> sphere = boost::static_pointer_cast<urdf::Sphere>(geometry_);
          if (name == "Radius (m)")
            item->setValue(sphere->radius);
        }
        else if (geometry_->type == urdf::Geometry::MESH)
        {
          boost::shared_ptr<urdf::Mesh> mesh = boost::static_pointer_cast<urdf::Mesh>(geometry_);
          if (name == "File Name")
            item->setValue(QString::fromStdString(mesh->filename));
          else if (name == "X")
            item->setValue(mesh->scale.x);
          else if (name == "Y")
            item->setValue(mesh->scale.y);
          else if (name == "Z")
            item->setValue(mesh->scale.z);
        }
      }
    }
    loading_ = false;
  }

  void LinkGeometryProperty::loadFactoryForManager(boost::shared_ptr<QtTreePropertyBrowser> &property_editor)
  {
    property_editor->setFactoryForManager(manager_, factory_);
  }

  void LinkGeometryProperty::onValueChanged(QtProperty *property, const QVariant &val)
  {
    if (loading_)
      return;

    QString name = property->propertyName();
    if (name == "Type")
    {
      // if type is changed need to add functionality to remove current geometry and add new one
      switch (val.toInt()) //{SPHERE, BOX, CYLINDER, MESH}
      {
      case 0:
        geometry_->type = urdf::Geometry::SPHERE;
        break;
      case 1:
        geometry_->type = urdf::Geometry::BOX;
        break;
      case 2:
        geometry_->type = urdf::Geometry::CYLINDER;
        break;
      case 3:
        geometry_->type = urdf::Geometry::MESH;
        break;
      }
    }
    else
    {
      if (geometry_->type == urdf::Geometry::BOX)
      {
        boost::shared_ptr<urdf::Box> box = boost::static_pointer_cast<urdf::Box>(geometry_);
        if (name == "Length X (m)")
          box->dim.x = val.toDouble();
        else if (name == "Length Y (m)")
          box->dim.y = val.toDouble();
        else if (name == "Length Z (m)")
          box->dim.z = val.toDouble();
      }
      else if (geometry_->type == urdf::Geometry::CYLINDER)
      {
        boost::shared_ptr<urdf::Cylinder> cylinder = boost::static_pointer_cast<urdf::Cylinder>(geometry_);
        if (name == "Radius (m)")
          cylinder->radius = val.toDouble();
        else if (name == "Length (m)")
          cylinder->length = val.toDouble();
      }
      else if (geometry_->type == urdf::Geometry::SPHERE)
      {
        boost::shared_ptr<urdf::Sphere> sphere = boost::static_pointer_cast<urdf::Sphere>(geometry_);
        if (name == "Radius (m)")
          sphere->radius = val.toDouble();
      }
      else if (geometry_->type == urdf::Geometry::MESH)
      {
        boost::shared_ptr<urdf::Mesh> mesh = boost::static_pointer_cast<urdf::Mesh>(geometry_);
        if (name == "File Name")
          mesh->filename = val.toString().toStdString();
        else if (name == "X")
          mesh->scale.x = val.toDouble();
        else if (name == "Y")
          mesh->scale.y = val.toDouble();
        else if (name == "Z")
          mesh->scale.z = val.toDouble();
      }
    }

    emit LinkGeometryProperty::valueChanged(property, val);
  }

  // Link Collision Property
  LinkCollisionProperty::LinkCollisionProperty(boost::shared_ptr<urdf::Collision> collision): collision_(collision), manager_(new QtVariantPropertyManager()), factory_(new QtVariantEditorFactory())
  {
    loading_ = true;
    QtVariantProperty *item;
    double p_norm, r_norm;
    urdf::Pose origin;

    QObject::connect(manager_, SIGNAL(valueChanged(QtProperty *, const QVariant &)),
              this, SLOT(onValueChanged(QtProperty *, const QVariant &)));

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
      QObject::connect(origin_property_.get(), SIGNAL(valueChanged(QtProperty *, const QVariant &)),
                this, SLOT(onChildValueChanged(QtProperty *, const QVariant &)));

      top_item_->addSubProperty(origin_property_->getTopItem());
    }

    if (collision_->geometry)
    {
      geometry_property_.reset(new LinkGeometryProperty(collision_->geometry));
      QObject::connect(geometry_property_.get(), SIGNAL(valueChanged(QtProperty *, const QVariant &)),
                this, SLOT(onChildValueChanged(QtProperty *, const QVariant &)));

      top_item_->addSubProperty(geometry_property_->getTopItem());
    }

    loading_ = false;
  }

   bool LinkCollisionProperty::hasOriginProperty()
  {
    return (origin_property_ != NULL);
  }

  void LinkCollisionProperty::createOriginProperty()
  {
    if (!origin_property_)
    {
      origin_property_.reset(new OriginProperty(collision_->origin));
      top_item_->addSubProperty(origin_property_->getTopItem());
    }
  }
  
  /*!
   *@brief Checks if geometry property exists
   *@returns geometry property exists
   */
  bool LinkCollisionProperty::hasGeometryProperty()
  {
    return (geometry_property_ != NULL);
  }

  /*!
   * @brief Creates the geometry property 
   */
  void LinkCollisionProperty::createGeometryProperty()
  {
    if (!geometry_property_)
    {
      collision_->geometry.reset(new urdf::Geometry());  //Create the URDF Geometry element 
      geometry_property_.reset(new LinkGeometryProperty(collision_->geometry));
      top_item_->addSubProperty(geometry_property_->getTopItem());
    }
  }
  
  
  
  LinkCollisionProperty::~LinkCollisionProperty()
  {
    delete manager_;
    delete factory_;
  }

  void LinkCollisionProperty::loadData()
  {
    loading_ = true;
    QtVariantProperty *item;
    QString name;
    QList<QtProperty *> sub_items = top_item_->subProperties();
    for (int i = 0; i < sub_items.length(); ++i)
    {
      item = static_cast<QtVariantProperty *>(sub_items[i]);
      name = item->propertyName();

      if (name == "Name")
        item->setValue(QString::fromStdString(collision_->group_name));
    }

    if (origin_property_)
      origin_property_->loadData();

    if (geometry_property_)
      geometry_property_->loadData();

    loading_ = false;
  }

  void LinkCollisionProperty::loadFactoryForManager(boost::shared_ptr<QtTreePropertyBrowser> &property_editor)
  {
    property_editor->setFactoryForManager(manager_, factory_);
    if (origin_property_)
      origin_property_->loadFactoryForManager(property_editor);

    if (geometry_property_)
      geometry_property_->loadFactoryForManager(property_editor);

  }

  void LinkCollisionProperty::onValueChanged(QtProperty *property, const QVariant &val)
  {
    if (loading_)
      return;

    QString name = property->propertyName();
    if (name == "Name")
      collision_->group_name = val.toString().toStdString();

    emit LinkCollisionProperty::valueChanged(property, val);
  }

  void LinkCollisionProperty::onChildValueChanged(QtProperty *property, const QVariant &val)
  {
    if (loading_)
      return;

    emit LinkCollisionProperty::valueChanged(property, val);
  }

  // Link Material Property
  LinkNewMaterialProperty::LinkNewMaterialProperty(boost::shared_ptr<urdf::Material> material): material_(material), manager_(new QtVariantPropertyManager()), factory_(new QtVariantEditorFactory())
  {
    loading_ = true;
    QtVariantProperty *item;

    QObject::connect(manager_, SIGNAL(valueChanged(QtProperty *, const QVariant &)),
              this, SLOT(onValueChanged(QtProperty *, const QVariant &)));

    top_item_ = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), tr("Material"));

    item = manager_->addProperty(QVariant::String, tr("Name"));
    top_item_->addSubProperty(item);

    item = manager_->addProperty(QVariant::Color, tr("Color"));
    top_item_->addSubProperty(item);

    item = manager_->addProperty(QVariant::String, tr("Texture"));
    top_item_->addSubProperty(item);

    loading_ = false;
  }

  LinkNewMaterialProperty::~LinkNewMaterialProperty()
  {
    delete manager_;
    delete factory_;
  }

  void LinkNewMaterialProperty::loadData()
  {
    loading_ = true;
    QtVariantProperty *item;
    QString name;
    QList<QtProperty *> sub_items = top_item_->subProperties();
    for (int i = 0; i < sub_items.length(); ++i)
    {
      item = static_cast<QtVariantProperty *>(sub_items[i]);
      name = item->propertyName();

      if (name == "Name")
        item->setValue(QString::fromStdString(material_->name));
      else if (name == "Color")
        item->setValue(QColor::fromRgba(qRgba(material_->color.r * 255.0, material_->color.g * 255.0, material_->color.b * 255.0, material_->color.a * 255.0)));
      else if (name == "Texture")
        item->setValue(QString::fromStdString(material_->texture_filename));
    }
    loading_ = false;
  }

  void LinkNewMaterialProperty::loadFactoryForManager(boost::shared_ptr<QtTreePropertyBrowser> &property_editor)
  {
    property_editor->setFactoryForManager(manager_, factory_);
  }

  void LinkNewMaterialProperty::onValueChanged(QtProperty *property, const QVariant &val)
  {
    if (loading_)
      return;

    QString name = property->propertyName();
    if (name == "Name")
      material_->name = val.toString().toStdString();
    else if (name == "Red")
      material_->color.r = val.toDouble()/255.0;
    else if (name == "Green")
      material_->color.g = val.toDouble()/255.0;
    else if (name == "Blue")
      material_->color.b = val.toDouble()/255.0;
    else if (name == "Alpha")
      material_->color.a = val.toDouble()/255.0;
    else if (name == "Texture")
      material_->texture_filename = val.toString().toStdString();

    emit LinkNewMaterialProperty::valueChanged(property, val);
  }

  // Link Visual Property
  LinkVisualProperty::LinkVisualProperty(boost::shared_ptr<urdf::Visual> visual): visual_(visual), manager_(new QtVariantPropertyManager()), factory_(new QtVariantEditorFactory())
  {
    loading_ = true;
    QtVariantProperty *item;
    double p_norm, r_norm;
    urdf::Pose origin;

    QObject::connect(manager_, SIGNAL(valueChanged(QtProperty *, const QVariant &)),
              this, SLOT(onValueChanged(QtProperty *, const QVariant &)));

    top_item_ = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), tr("Visual"));
   
    item = manager_->addProperty(QVariant::String, tr("Name"));
    top_item_->addSubProperty(item);
    
    item = manager_->addProperty(QVariant::Bool, tr("Show in Editor"));
    QObject::connect(
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
      QObject::connect(origin_property_.get(), SIGNAL(valueChanged(QtProperty *, const QVariant &)),
                this, SLOT(onChildValueChanged(QtProperty *, const QVariant &)));

      top_item_->addSubProperty(origin_property_->getTopItem());
    }

    if (visual_->material)
    {
      new_material_property_.reset(new LinkNewMaterialProperty(visual_->material));
      QObject::connect(new_material_property_.get(), SIGNAL(valueChanged(QtProperty *, const QVariant &)),
                this, SLOT(onChildValueChanged(QtProperty *, const QVariant &)));

      top_item_->addSubProperty(new_material_property_->getTopItem());
    }

    if (visual_->geometry)
    {
      geometry_property_.reset(new LinkGeometryProperty(visual_->geometry));
      QObject::connect(geometry_property_.get(), SIGNAL(valueChanged(QtProperty *, const QVariant &)),
                this, SLOT(onChildValueChanged(QtProperty *, const QVariant &)));

      top_item_->addSubProperty(geometry_property_->getTopItem());
    }
    else
    {
      //TODO: need to create one since it is not optional
    }
    loading_ = false;
  }
  
  bool LinkVisualProperty::hasOriginProperty()
  {
    return (origin_property_ != NULL);
  }

  void LinkVisualProperty::createOriginProperty()
  {
    if (!origin_property_)
    {
      origin_property_.reset(new OriginProperty(visual_->origin));
      top_item_->addSubProperty(origin_property_->getTopItem());
    }
  }
  
  /*!
   *@brief Checks if geometry property exists
   *@returns geometry property exists
   */
  bool LinkVisualProperty::hasGeometryProperty()
  {
    return (geometry_property_ != NULL);
  }

  /*!
   * @brief Creates the geometry property 
   */
  void LinkVisualProperty::createGeometryProperty()
  {
    if (!geometry_property_)
    {
      visual_->geometry.reset(new urdf::Geometry());  //Create the URDF Geometry element 
      geometry_property_.reset(new LinkGeometryProperty(visual_->geometry));
      top_item_->addSubProperty(geometry_property_->getTopItem());
    }
  }
  
  /*!
   *@brief Checks if Material property exists
   *@returns material property exists
   */
  bool LinkVisualProperty::hasMaterialProperty()
  {
    return (new_material_property_ != NULL);
  }

  /*!
   * @brief Creates the material property 
   */
  void LinkVisualProperty::createMaterialProperty()
  {
    if (!new_material_property_)
    {
       visual_->material.reset(new urdf::Material());  //Create the URDF Material element 
      new_material_property_.reset(new LinkNewMaterialProperty(visual_->material));
      top_item_->addSubProperty(new_material_property_->getTopItem());
    }
  }
  
  
  

  LinkVisualProperty::~LinkVisualProperty()
  {
    delete manager_;
    delete factory_;
  }

  void LinkVisualProperty::loadData()
  {
    loading_ = true;
    QtVariantProperty *item;
    QString name;
    QList<QtProperty *> sub_items = top_item_->subProperties();
    for (int i = 0; i < sub_items.length(); ++i)
    {
      item = static_cast<QtVariantProperty *>(sub_items[i]);
      name = item->propertyName();
      if (name == "Name")
        item->setValue(QString::fromStdString(visual_->group_name));
    }

    if (origin_property_)
      origin_property_->loadData();

    if (new_material_property_)
      new_material_property_->loadData();

    if (geometry_property_)
      geometry_property_->loadData();

    loading_ = false;
  }

  void LinkVisualProperty::loadFactoryForManager(boost::shared_ptr<QtTreePropertyBrowser> &property_editor)
  {
    property_editor->setFactoryForManager(manager_, factory_);
    if (origin_property_)
      origin_property_->loadFactoryForManager(property_editor);

    if (new_material_property_)
      new_material_property_->loadFactoryForManager(property_editor);

    if (geometry_property_)
      geometry_property_->loadFactoryForManager(property_editor);

  }

  void LinkVisualProperty::onValueChanged(QtProperty *property, const QVariant &val)
  {
    if (loading_)
      return;

    QString name = property->propertyName();
    if (name == "Name")
      visual_->group_name = val.toString().toStdString();

    emit LinkVisualProperty::valueChanged(property, val);
  }

  void LinkVisualProperty::onChildValueChanged(QtProperty *property, const QVariant &val)
  {
    if (loading_)
      return;

    emit LinkVisualProperty::valueChanged(property, val);
  }

  // Link Inertial Property
  LinkInertialProperty::LinkInertialProperty(boost::shared_ptr<urdf::Inertial> inertial): inertial_(inertial), manager_(new QtVariantPropertyManager()), factory_(new QtVariantEditorFactory())
  {
    loading_ = true;
    QtVariantProperty *item;
    QtVariantProperty *sub_item;
    double p_norm, r_norm;
    urdf::Pose origin;

    QObject::connect(manager_, SIGNAL(valueChanged(QtProperty *, const QVariant &)),
              this, SLOT(onValueChanged(QtProperty *, const QVariant &)));

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
      QObject::connect(origin_property_.get(), SIGNAL(valueChanged(QtProperty *, const QVariant &)),
                this, SLOT(onChildValueChanged(QtProperty *, const QVariant &)));

      top_item_->addSubProperty(origin_property_->getTopItem());
    }
    loading_ = false;
  }

  LinkInertialProperty::~LinkInertialProperty()
  {
    delete manager_;
    delete factory_;
  }

  void LinkInertialProperty::loadData()
  {
    loading_ = true;
    QtVariantProperty *item;
    QString name;

    //Get Mass, since its the first one
    item = static_cast<QtVariantProperty *>(top_item_->subProperties()[0]);
    name = item->propertyName();
      if (name == "Mass (kg)")
        item->setValue(inertial_->mass);

    //Remaining sub-properties are for Inertial values
    QList<QtProperty *> sub_items = top_item_->subProperties()[1]->subProperties();
    for (int i = 0; i < sub_items.length(); ++i)
    {
      item = static_cast<QtVariantProperty *>(sub_items[i]);
      name = item->propertyName();
      if (name == "Mass (kg)")
        item->setValue(inertial_->mass);
      else if (name == "Ixx")
        item->setValue(inertial_->ixx);
      else if (name == "Iyy")
        item->setValue(inertial_->iyy);
      else if (name == "Izz")
        item->setValue(inertial_->izz);
      else if (name == "Ixy")
        item->setValue(inertial_->ixy);
      else if (name == "Ixz")
        item->setValue(inertial_->ixz);
      else if (name == "Iyz")
        item->setValue(inertial_->iyz);
    }

    if (origin_property_)
      origin_property_->loadData();

    loading_ = false;
  }

  bool LinkInertialProperty::hasOriginProperty()
  {
    return (origin_property_ != NULL);
  }

  void LinkInertialProperty::createOriginProperty()
  {
    if (!origin_property_)
    {
      origin_property_.reset(new OriginProperty(inertial_->origin));
      top_item_->addSubProperty(origin_property_->getTopItem());
    }
  }

  void LinkInertialProperty::loadFactoryForManager(boost::shared_ptr<QtTreePropertyBrowser> &property_editor)
  {
    property_editor->setFactoryForManager(manager_, factory_);
    if (origin_property_)
      origin_property_->loadFactoryForManager(property_editor);
  }

  void LinkInertialProperty::onValueChanged(QtProperty *property, const QVariant &val)
  {
    if (loading_)
      return;

    QString name = property->propertyName();
    if (name == "Mass (kg)")
      inertial_->mass = val.toDouble();
    else if (name == "Ixx")
      inertial_->ixx = val.toDouble();
    else if (name == "Iyy")
      inertial_->iyy = val.toDouble();
    else if (name == "Izz")
      inertial_->izz = val.toDouble();
    else if (name == "Ixy")
      inertial_->ixy = val.toDouble();
    else if (name == "Ixz")
      inertial_->ixz = val.toDouble();
    else if (name == "Iyz")
      inertial_->iyz = val.toDouble();

    emit LinkInertialProperty::valueChanged(property, val);
  }

  void LinkInertialProperty::onChildValueChanged(QtProperty *property, const QVariant &val)
  {
    if (loading_)
      return;

    emit LinkInertialProperty::valueChanged(property, val);
  }

  // Link Property
  LinkProperty::LinkProperty(boost::shared_ptr<urdf::Link> link):link_(link), manager_(new QtVariantPropertyManager()), factory_(new QtVariantEditorFactory())
  {
    loading_ = true;
    QObject::connect(manager_, SIGNAL(valueChanged(QtProperty *, const QVariant &)),
              this, SLOT(onValueChanged(QtProperty *, const QVariant &)));

    //top_item_ = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), tr("Link Properties"));

    name_item_ = manager_->addProperty(QVariant::String, tr("Name"));

    if (link_->inertial)
    {
      inertial_property_.reset(new LinkInertialProperty(link_->inertial));
      QObject::connect(inertial_property_.get(), SIGNAL(valueChanged(QtProperty *, const QVariant &)),
                this, SLOT(onChildValueChanged(QtProperty *, const QVariant &)));
    }

    if (link_->visual)
    {
      visual_property_.reset(new LinkVisualProperty(link_->visual));
      QObject::connect(visual_property_.get(), SIGNAL(valueChanged(QtProperty *, const QVariant &)),
                this, SLOT(onChildValueChanged(QtProperty *, const QVariant &)));
    }

    if (link_->collision)
    {
      collision_property_.reset(new LinkCollisionProperty(link_->collision));
      QObject::connect(collision_property_.get(), SIGNAL(valueChanged(QtProperty *, const QVariant &)),
                this, SLOT(onChildValueChanged(QtProperty *, const QVariant &)));
    }

    loading_ = true;
  }

  LinkProperty::~LinkProperty()
  {
    delete manager_;
    delete factory_;
  }

  void LinkProperty::loadData()
  {
    loading_ = true;
    name_item_->setValue(QString::fromStdString(link_->name));

    if (inertial_property_)
      inertial_property_->loadData();

    if (visual_property_)
      visual_property_->loadData();

    if (collision_property_)
      collision_property_->loadData();

    loading_ = false;
  }

  void LinkProperty::loadProperty(boost::shared_ptr<QtTreePropertyBrowser> property_editor)
  {
    loadData();
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

  bool LinkProperty::hasInertialProperty()
  {
    return (inertial_property_ != NULL);
  }

  void LinkProperty::createInertialProperty()
  {
    if(!link_->inertial)
    {
      link_->inertial.reset(new urdf::Inertial());
      inertial_property_.reset(new LinkInertialProperty(link_->inertial));
    }
  }

  LinkInertialPropertyPtr LinkProperty::getInertialProperty()
  {
    return inertial_property_;
  }
  
  
  /*!
   *@brief Checks if the Link has a visual property defined
   * 
   *@return returns true of visual property defined
   */
  bool LinkProperty::hasVisualProperty()
  {
    return (visual_property_ != NULL);
  }

  /*!
   *@brief Creates the visual property
   * 
   */
  void LinkProperty::createVisualProperty()
  {
    if(!link_->visual)
    {
      link_->visual.reset(new urdf::Visual());
      visual_property_.reset(new LinkVisualProperty(link_->visual));
    }
  }
  
   /*!
   *@brief Get the visual property Object
   * 
   *@return LinkVisualPropertyPtr
   */
  LinkVisualPropertyPtr LinkProperty::getVisualProperty()
  {
    return visual_property_;
  }
  
  /*!
   *@brief Checks if the Link has a collision property defined
   * 
   *@return returns true of collision property defined
   */
  bool LinkProperty::hasCollisionProperty()
  {
    return (collision_property_ != NULL);
  }

  /*!
   *@brief Creates the collision property
   * 
   */
  void LinkProperty::createCollisionProperty()
  {
    if(!link_->collision)
    {
      link_->collision.reset(new urdf::Collision());
      collision_property_.reset(new LinkCollisionProperty(link_->collision));
    }
  }
  
   /*!
   *@brief Get the collision property Object
   * 
   *@return LinkCollisionPropertyPtr
   */
  LinkCollisionPropertyPtr LinkProperty::getCollisionProperty()
  {
    return collision_property_;
  }
  

  void LinkProperty::onValueChanged(QtProperty *property, const QVariant &val)
  {
    if (loading_)
      return;

    QString name = property->propertyName();

    if (name == "Name")
    {
      std::string link_name = val.toString().toStdString();
      link_->name = link_name;

      if (link_->parent_joint)
      {
        link_->parent_joint->child_link_name = link_name;
      }

      for (unsigned int i = 0; i < link_->child_links.size(); ++i)
      {
        link_->child_links[i]->parent_joint->parent_link_name = link_name;
      }

      emit LinkProperty::linkNameChanged(this, val);
    }
    emit LinkProperty::valueChanged();
  }

  void LinkProperty::onChildValueChanged(QtProperty *property, const QVariant &val)
  {
    if (loading_)
      return;

    emit LinkProperty::valueChanged();
  }
}

