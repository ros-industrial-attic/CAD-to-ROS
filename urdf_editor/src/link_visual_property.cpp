
#include <qttreepropertybrowser.h>
#include <qtvariantproperty.h>

#include <urdf_editor/link_visual_property.h>
#include <urdf_editor/link_new_material_property.h>
#include <urdf_editor/link_geometry_property.h>
#include <urdf_editor/origin_property.h>
#include <urdf_editor/common.h>
#include <urdf_model/link.h>

namespace urdf_editor
{
  LinkVisualProperty::LinkVisualProperty(urdf::VisualSharedPtr visual): visual_(visual), manager_(new QtVariantPropertyManager()), factory_(new QtVariantEditorFactory())
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

    
    origin = visual_->origin;
    p_norm = origin.position.x * origin.position.x;
    p_norm += origin.position.y * origin.position.y;
    p_norm += origin.position.z * origin.position.z;
    r_norm = origin.rotation.x * origin.rotation.x;
    r_norm += origin.rotation.y * origin.rotation.y;
    r_norm += origin.rotation.z * origin.rotation.z;
    if (p_norm > 0.0 || r_norm > 0.0)
      createOriginProperty();

    if (visual_->material)
      createMaterialProperty();

    // The geometry property is not optional
    createGeometryProperty();

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
      QObject::connect(origin_property_.get(), SIGNAL(valueChanged(QtProperty *, const QVariant &)),
                this, SLOT(onChildValueChanged(QtProperty *, const QVariant &)));

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
   *@brief Get the geometry property Object
   * 
   *@return LinkGeometryPropertySharedPtr
   */
  LinkGeometryPropertySharedPtr LinkVisualProperty::getGeometryProperty()
  {
    return geometry_property_;
  }

  /*!
   * @brief Creates the geometry property 
   */
  void LinkVisualProperty::createGeometryProperty()
  {
    if (!geometry_property_)
    {
      if (visual_->geometry == NULL)
      {
        urdf::SphereSharedPtr geometry(new urdf::Sphere());
        geometry->radius = 0.1;
        visual_->geometry = geometry;  //Create the URDF Geometry element
        visual_->geometry->type = visual_->geometry->SPHERE;
      }

      geometry_property_.reset(new LinkGeometryProperty(visual_->geometry));
      QObject::connect(geometry_property_.get(), SIGNAL(valueChanged(QtProperty *, const QVariant &)),
                this, SLOT(onChildValueChanged(QtProperty *, const QVariant &)));

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
      if (visual_->material == NULL)
        visual_->material.reset(new urdf::Material());  //Create the URDF Material element

      new_material_property_.reset(new LinkNewMaterialProperty(visual_->material));
      QObject::connect(new_material_property_.get(), SIGNAL(valueChanged(QtProperty *, const QVariant &)),
                       this, SLOT(onChildValueChanged(QtProperty *, const QVariant &)));

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

  void LinkVisualProperty::removeSubProperties()
  {
    QList<QtProperty *> sub_items = top_item_->subProperties();
    for (int i = 0; i < sub_items.length(); ++i)
        top_item_->removeSubProperty(sub_items[i]);
  }

  void LinkVisualProperty::loadFactoryForManager(QtTreePropertyBrowserSharedPtr& property_editor)
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

    if (property->propertyName() == "Type")
    {
        visual_->geometry = geometry_property_->getGeometry();
        emit LinkVisualProperty::geometryChanged(val.toInt());
    }

    emit LinkVisualProperty::valueChanged(property, val);


  }
}
