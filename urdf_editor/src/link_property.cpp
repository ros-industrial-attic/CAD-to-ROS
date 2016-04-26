
#include <string>

#include <qttreepropertybrowser.h>
#include <qtvariantproperty.h>

#include <urdf_model/link.h>

#include <urdf_editor/link_collision_property.h>
#include <urdf_editor/link_geometry_property.h>
#include <urdf_editor/link_inertial_property.h>
#include <urdf_editor/link_new_material_property.h>
#include <urdf_editor/link_visual_property.h>
#include <urdf_editor/link_property.h>


namespace urdf_editor
{
  LinkProperty::LinkProperty(urdf::LinkSharedPtr link):link_(link), manager_(new QtVariantPropertyManager()), factory_(new QtVariantEditorFactory())
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
      QObject::connect(visual_property_.get(), SIGNAL(geometryChanged(int)),
                this, SLOT(onVisualGeometryChanged(int)));
    }

    if (link_->collision)
    {
      collision_property_.reset(new LinkCollisionProperty(link_->collision));
      QObject::connect(collision_property_.get(), SIGNAL(valueChanged(QtProperty *, const QVariant &)),
                this, SLOT(onChildValueChanged(QtProperty *, const QVariant &)));
      QObject::connect(collision_property_.get(), SIGNAL(geometryChanged(int)),
                this, SLOT(onCollisionGeometryChanged(int)));
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

  LinkInertialPropertySharedPtr LinkProperty::getInertialProperty()
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
   *@return LinkVisualPropertySharedPtr
   */
  LinkVisualPropertySharedPtr LinkProperty::getVisualProperty()
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
   *@return LinkCollisionPropertySharedPtr
   */
  LinkCollisionPropertySharedPtr LinkProperty::getCollisionProperty()
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

  void LinkProperty::onVisualGeometryChanged(int type)
  {
    if (loading_)
      return;

    switch (type) //{SPHERE, BOX, CYLINDER, MESH}
    {
    case 0:
      {
      link_->visual->geometry.reset(new urdf::Sphere);
      link_->visual->geometry->type = urdf::Geometry::SPHERE;
      boost::shared_ptr<urdf::Sphere> sphere = boost::static_pointer_cast<urdf::Sphere>(link_->visual->geometry);
      sphere->radius = 0.01; 
      break;
      }
    case 1:
      {
      link_->visual->geometry.reset(new urdf::Box);
      link_->visual->geometry->type = urdf::Geometry::BOX;
      boost::shared_ptr<urdf::Box> box = boost::static_pointer_cast<urdf::Box>(link_->visual->geometry);
      box->dim.x = 0.01;
      box->dim.y = 0.01;
      box->dim.z = 0.01;      
      break;
      }
    case 2:
      {
      link_->visual->geometry.reset(new urdf::Cylinder);
      link_->visual->geometry->type = urdf::Geometry::CYLINDER;
      boost::shared_ptr<urdf::Cylinder> cylinder = boost::static_pointer_cast<urdf::Cylinder>(link_->visual->geometry);
      cylinder->radius = 0.01;
      cylinder->length = 0.01;
      break;
      }
    case 3:
      {
      link_->visual->geometry.reset(new urdf::Mesh);
      link_->visual->geometry->type = urdf::Geometry::MESH;    
      boost::shared_ptr<urdf::Mesh> mesh = boost::static_pointer_cast<urdf::Mesh>(link_->visual->geometry);
      mesh->scale.x = 1.0;
      mesh->scale.y = 1.0;
      mesh->scale.z = 1.0;
      break;
      }
    }

    visual_property_->getGeometryProperty()->setGeometry(link_->visual->geometry); 

    emit LinkProperty::valueChanged();
  }

  void LinkProperty::onCollisionGeometryChanged(int type)
  {
    if (loading_)
      return;

    switch (type) //{SPHERE, BOX, CYLINDER, MESH}
    {
    case 0:
      {
      link_->collision->geometry.reset(new urdf::Sphere);
      link_->collision->geometry->type = urdf::Geometry::SPHERE;
      boost::shared_ptr<urdf::Sphere> sphere = boost::static_pointer_cast<urdf::Sphere>(link_->collision->geometry);
      sphere->radius = 0.01; 
      break;
      }
    case 1:
      {
      link_->collision->geometry.reset(new urdf::Box);
      link_->collision->geometry->type = urdf::Geometry::BOX;
      boost::shared_ptr<urdf::Box> box = boost::static_pointer_cast<urdf::Box>(link_->collision->geometry);
      box->dim.x = 0.01;
      box->dim.y = 0.01;
      box->dim.z = 0.01;      
      break;
      }
    case 2:
      {
      link_->collision->geometry.reset(new urdf::Cylinder);
      link_->collision->geometry->type = urdf::Geometry::CYLINDER;
      boost::shared_ptr<urdf::Cylinder> cylinder = boost::static_pointer_cast<urdf::Cylinder>(link_->collision->geometry);
      cylinder->radius = 0.01;
      cylinder->length = 0.01;
      break;
      }
    case 3:
      {
      link_->collision->geometry.reset(new urdf::Mesh);
      link_->collision->geometry->type = urdf::Geometry::MESH;    
      boost::shared_ptr<urdf::Mesh> mesh = boost::static_pointer_cast<urdf::Mesh>(link_->collision->geometry);
      mesh->scale.x = 1.0;
      mesh->scale.y = 1.0;
      mesh->scale.z = 1.0;
      break;
      }
    }

    collision_property_->getGeometryProperty()->setGeometry(link_->collision->geometry); 

    emit LinkProperty::valueChanged();
  }
}
