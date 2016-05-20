
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
      createInertialProperty();

    if (link_->visual)
      createVisualPropertyHelper(link_->visual);
    else if (!link_->visual_array.empty())
      for (int i; i < link_->visual_array.size(); i++)
        createVisualPropertyHelper(link->visual_array[i]);

    if (link_->collision)
      createCollisionPropertyHelper(link_->collision);
    else if (!link_->collision_array.empty())
      for (int i; i < link_->collision_array.size(); i++)
        createCollisionPropertyHelper(link->collision_array[i]);

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

    if (!visual_property_.empty())
      foreach (LinkVisualPropertySharedPtr visual, visual_property_)
        visual->loadData();

    if (!collision_property_.empty())
      foreach (LinkCollisionPropertySharedPtr collision, collision_property_)
        collision->loadData();

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

    if (!visual_property_.empty())
    {
      foreach (LinkVisualPropertySharedPtr visual, visual_property_)
      {
        visual->loadFactoryForManager(property_editor);
        property_editor->addProperty(visual->getTopItem());
      }
    }

    if (!collision_property_.empty())
    {
      foreach (LinkCollisionPropertySharedPtr collision, collision_property_)
      {
        collision->loadFactoryForManager(property_editor);
        property_editor->addProperty(collision->getTopItem());
      }
    }
  }

  bool LinkProperty::hasInertialProperty()
  {
    return (inertial_property_ != NULL);
  }

  void LinkProperty::createInertialProperty()
  {
    if (link_->inertial == NULL)
      link_->inertial.reset(new urdf::Inertial());

    inertial_property_.reset(new LinkInertialProperty(link_->inertial));
    QObject::connect(inertial_property_.get(), SIGNAL(valueChanged(QtProperty *, const QVariant &)),
              this, SLOT(onChildValueChanged(QtProperty *, const QVariant &)));

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
    return (!visual_property_.empty());
  }

  /*!
   *@brief Creates the visual property
   * 
   */
  void LinkProperty::createVisualProperty()
  {
    urdf::VisualSharedPtr visual(new urdf::Visual());
    if (link_->visual != NULL)
    {
      link_->visual_array.clear();
      link_->visual_array.push_back(link_->visual);
      link_->visual_array.push_back(visual);
      createVisualPropertyHelper(link_->visual_array.back());

      link_->visual.reset();
    }
    else if (link_->visual == NULL & link_->visual_array.empty())
    {
      link_->visual = visual;
      link_->visual_array.push_back(visual);
      createVisualPropertyHelper(link_->visual);
    }
    else
    {
      link_->visual_array.push_back(visual);
      createVisualPropertyHelper(link_->visual_array.back());
    }
  }

  /*!
   *@brief Creates the visual property helper function
   *
   */
  void LinkProperty::createVisualPropertyHelper(urdf::VisualSharedPtr data)
  {
    visual_property_.push_back(LinkVisualPropertySharedPtr(new LinkVisualProperty(data)));
    QObject::connect(visual_property_.back().get(), SIGNAL(valueChanged(QtProperty *, const QVariant &)),
              this, SLOT(onChildValueChanged(QtProperty *, const QVariant &)));
    QObject::connect(visual_property_.back().get(), SIGNAL(geometryChanged(int)),
              this, SLOT(onVisualGeometryChanged(int)));

    // This is required because properties are changed during creation but connection are not setup until
    // after the object is created.
    emit LinkProperty::valueChanged(this);
  }
  
   /*!
   *@brief Get the list of visual property Objects
   * 
   *@return std::vector<LinkVisualPropertySharedPtr>
   */
  std::vector<LinkVisualPropertySharedPtr> LinkProperty::getVisualProperties()
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
    return (!collision_property_.empty());
  }

  /*!
   *@brief Creates the collision property
   * 
   */
  void LinkProperty::createCollisionProperty()
  {
    urdf::CollisionSharedPtr collision(new urdf::Collision());
    if (link_->collision != NULL)
    {
      link_->collision_array.clear();
      link_->collision_array.push_back(link_->collision);
      link_->collision_array.push_back(collision);
      createCollisionPropertyHelper(link_->collision_array.back());

      link_->collision.reset();
    }
    else if (link_->collision == NULL & link_->collision_array.empty())
    {
      link_->collision = collision;
      link_->collision_array.push_back(collision);
      createCollisionPropertyHelper(link_->collision);
    }
    else
    {
      link_->collision_array.push_back(collision);
      createCollisionPropertyHelper(link_->collision_array.back());
    }
  }

  /*!
   *@brief Creates the collision property helper function
   *
   */
  void LinkProperty::createCollisionPropertyHelper(urdf::CollisionSharedPtr data)
  {
    collision_property_.push_back(LinkCollisionPropertySharedPtr(new LinkCollisionProperty(data)));
    QObject::connect(collision_property_.back().get(), SIGNAL(valueChanged(QtProperty *, const QVariant &)),
              this, SLOT(onChildValueChanged(QtProperty *, const QVariant &)));
    QObject::connect(collision_property_.back().get(), SIGNAL(geometryChanged(int)),
              this, SLOT(onCollisionGeometryChanged(int)));

    // This is required because properties are changed during creation but connection are not setup until
    // after the object is created.
    emit LinkProperty::valueChanged(this);
  }
  
   /*!
   *@brief Get the list of collision property Objects
   * 
   *@return std::vector<LinkCollisionPropertySharedPtr>
   */
  std::vector<LinkCollisionPropertySharedPtr> LinkProperty::getCollisionProperties()
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
    emit LinkProperty::valueChanged(this);
  }

  void LinkProperty::onChildValueChanged(QtProperty *property, const QVariant &val)
  {
    if (loading_)
      return;

    emit LinkProperty::valueChanged(this);
  }

  /*! Get the link name */
  QString LinkProperty::getName()
  {
    return QString::fromStdString(link_->name);
  }

  void LinkProperty::onVisualGeometryChanged(int type)
  {
    if (loading_)
      return;

    emit LinkProperty::valueChanged(this);
  }

  void LinkProperty::onCollisionGeometryChanged(int type)
  {
    if (loading_)
      return;

    emit LinkProperty::valueChanged(this);
  }
}
