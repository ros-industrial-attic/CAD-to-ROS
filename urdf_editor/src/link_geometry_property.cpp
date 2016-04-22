
#include <qttreepropertybrowser.h>
#include <qtvariantproperty.h>

#include <urdf_editor/link_geometry_property.h>
#include <urdf_editor/common.h>

#include <urdf_model/link.h>


namespace urdf_editor
{
  LinkGeometryProperty::LinkGeometryProperty(urdf::GeometrySharedPtr geometry): geometry_(geometry), manager_(new QtVariantPropertyManager()), factory_(new QtVariantEditorFactory())
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

    std::cout<<"THE Geometry TYPE IS "<<geometry_->type<<std::endl;
    loading_ = false;
  }

  LinkGeometryProperty::~LinkGeometryProperty()
  {
    delete manager_;
    delete factory_;
  }

  void LinkGeometryProperty::loadData()
  {
    std::cout<<"LOADING Geometry data.........................."<<std::endl;
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

  void LinkGeometryProperty::loadFactoryForManager(QtTreePropertyBrowserSharedPtr& property_editor)
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
      
      // remove all the subproperties
      QList<QtProperty *> sub_items = top_item_->subProperties();
      for (int i = 0; i < sub_items.length(); ++i) 
        if (sub_items[i]->propertyName() != "Type")
          top_item_->removeSubProperty(sub_items[i]);

      // add appropriate subproperties back in
      QtVariantProperty *item;
      QtVariantProperty *sub_item;
      loading_ = true;
      switch (val.toInt()) //{SPHERE, BOX, CYLINDER, MESH}
      {
      case 0:
        {
        geometry_.reset(new urdf::Sphere);
        geometry_->type = urdf::Geometry::SPHERE;

        item = manager_->addProperty(QVariant::Double, tr("Radius (m)"));
        top_item_->addSubProperty(item);
        break;
        }
      case 1:
        {
        geometry_.reset(new urdf::Box);
        geometry_->type = urdf::Geometry::BOX;
        
        item = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), tr("Size"));
        sub_item = manager_->addProperty(QVariant::Double, tr("Length X (m)"));
        item->addSubProperty(sub_item);

        sub_item = manager_->addProperty(QVariant::Double, tr("Length Y (m)"));
        item->addSubProperty(sub_item);

        sub_item = manager_->addProperty(QVariant::Double, tr("Length Z (m)"));
        item->addSubProperty(sub_item);
        top_item_->addSubProperty(item);
        break;
        }
      case 2:
        {
        geometry_.reset(new urdf::Cylinder);
        geometry_->type = urdf::Geometry::CYLINDER;

        item = manager_->addProperty(QVariant::Double, tr("Radius (m)"));
        top_item_->addSubProperty(item);

        item = manager_->addProperty(QVariant::Double, tr("Length (m)"));
        top_item_->addSubProperty(item);
        break;
        }
      case 3:
        {
        geometry_.reset(new urdf::Mesh);
        geometry_->type = urdf::Geometry::MESH;

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
        break;
        }
      }
      loading_ = false;
    }
    else
    {

      if (geometry_->type == urdf::Geometry::BOX)
      {
        if (urdf::dynamic_pointer_cast<urdf::Box>(geometry_))
        {
          std::cout<<"was able to recast the geometry_ shape into box while EDITING VALUES!!!!!"<<std::endl;
        } else std::cout<<"WAS NOT able to recast the geometry_ shape into a box while EDITING VALUES!!!!"<<std::endl;

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
        if (urdf::dynamic_pointer_cast<urdf::Cylinder>(geometry_))
        {
          std::cout<<"was able to recast the geometry_ shape into cylinder while EDITING VALUES!!!!!"<<std::endl;
        } else std::cout<<"WAS NOT able to recast the geometry_ shape into a cylinder while EDITING VALUES!!!!"<<std::endl;

        boost::shared_ptr<urdf::Cylinder> cylinder = urdf::static_pointer_cast<urdf::Cylinder>(geometry_);
        if (name == "Radius (m)")
          cylinder->radius = val.toDouble();
        else if (name == "Length (m)")
          cylinder->length = val.toDouble();
      }
      else if (geometry_->type == urdf::Geometry::SPHERE)
      {
        if (urdf::dynamic_pointer_cast<urdf::Sphere>(geometry_))
        {
          std::cout<<"was able to recast the geometry_ shape into sphere while EDITING VALUES!!!!!"<<std::endl;
        } else std::cout<<"WAS NOT able to recast the geometry_ shape into a sphere while EDITING VALUES!!!!"<<std::endl;

        boost::shared_ptr<urdf::Sphere> sphere = boost::static_pointer_cast<urdf::Sphere>(geometry_);
        if (name == "Radius (m)")
          sphere->radius = val.toDouble();
      }
      else if (geometry_->type == urdf::Geometry::MESH)
      {
        if (urdf::dynamic_pointer_cast<urdf::Mesh>(geometry_))
        {
          std::cout<<"was able to recast the geometry_ shape into mesh while EDITING VALUES!!!!!"<<std::endl;
        } else std::cout<<"WAS NOT able to recast the geometry_ shape into a mesh while EDITING VALUES!!!!"<<std::endl;
        
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
      std::cout<<"CHANGED some type "<<geometry_->type<<std::endl;
    }

    emit LinkGeometryProperty::valueChanged(property, val);
  }
}