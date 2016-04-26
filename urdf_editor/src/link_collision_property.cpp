
#include <qttreepropertybrowser.h>
#include <qtvariantproperty.h>

#include <urdf_editor/link_collision_property.h>
#include <urdf_editor/link_geometry_property.h>
#include <urdf_editor/origin_property.h>
#include <urdf_editor/common.h>

#include <urdf_model/link.h>


namespace urdf_editor
{
  LinkCollisionProperty::LinkCollisionProperty(urdf::CollisionSharedPtr collision): collision_(collision), manager_(new QtVariantPropertyManager()), factory_(new QtVariantEditorFactory())
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
   *@brief Get the geometry property Object
   * 
   *@return LinkGeometryPropertySharedPtr
   */
  LinkGeometryPropertySharedPtr LinkCollisionProperty::getGeometryProperty()
  {
    return geometry_property_;
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

  void LinkCollisionProperty::loadFactoryForManager(QtTreePropertyBrowserSharedPtr& property_editor)
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
    
    if (property->propertyName() == "Type")
    {
      if (hasGeometryProperty())
        emit LinkCollisionProperty::geometryChanged();
    }
    else
    {
      emit LinkCollisionProperty::valueChanged(property, val);
    }
  }
}
