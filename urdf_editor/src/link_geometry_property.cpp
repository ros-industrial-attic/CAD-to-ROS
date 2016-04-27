
#include <qttreepropertybrowser.h>
#include <qtvariantproperty.h>
#include <qmenu.h>
#include <qaction.h>
#include <qfiledialog.h>

#include <urdf_editor/link_geometry_property.h>
#include <urdf_editor/common.h>
#include <urdf_model/link.h>

#include <ros/package.h>
#include <ros/console.h>

static const std::string PACKAGE_NAME = "urdf_builder";
static const std::map<std::string,std::string> SUPPORTED_MESH_EXTENSIONS_MAP = {{"Collada" ,"dae"},
                                                                                {"STL","stl"},
                                                                                {"Wavefront","obj"}};

static std::string getROSFormattedPath(const std::string& full_path)
{
  // determining file and path
  std::string pkg,local_path;
  std::size_t file_pos = full_path.find_last_of("/");
  std::string file_name = full_path.substr(file_pos+1);

  // finding ros package
  std::size_t start_pos, end_pos = file_pos;
  std::string partial_path = full_path.substr(0,end_pos);
  while(start_pos != std::string::npos)
  {
    start_pos = partial_path.find_last_of("/");
    if(start_pos ==  std::string::npos )
    {
      ROS_ERROR("The file %s is not located inside a ros package",file_name.c_str());
      return "";
    }

    pkg = partial_path.substr(start_pos+1);
    if(!ros::package::getPath(pkg).empty())
    {
      // found ros package
      break;
    }

    partial_path = partial_path.substr(0,start_pos);
    local_path = pkg + "/" + local_path;
  }

  std::string ros_formatted_path = "package://"+ pkg + "/" + local_path +  file_name;

  return ros_formatted_path;
}

namespace urdf_editor
{
  LinkGeometryProperty::LinkGeometryProperty(urdf::GeometrySharedPtr geometry):
      geometry_(geometry),
      manager_(new QtVariantPropertyManager()),
      factory_(new QtVariantEditorFactory()),
      mesh_path_("none"),
      browse_start_dir_(ros::package::getPath(PACKAGE_NAME))
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

    loadData();

    loading_ = false;
  }

  LinkGeometryProperty::~LinkGeometryProperty()
  {
    delete manager_;
    delete factory_;
  }

  void LinkGeometryProperty::loadMesh()
  {
    // create formats filter
    std::string filter;
    for(auto& p:SUPPORTED_MESH_EXTENSIONS_MAP)
    {
      filter = filter + p.first + " (*." + p.second + ");;";
    }
    filter = filter.substr(0,filter.length()-2);

    QString qpath = QFileDialog::getOpenFileName(0,QString("Open Mesh"),
                                                   QString::fromStdString(browse_start_dir_),
                                                   QString::fromStdString(filter));

    if(qpath.isEmpty())
    {
      return;
    }

    std::string ros_path = getROSFormattedPath(qpath.toStdString());
    ROS_INFO_STREAM("ROS mesh path "<<ros_path);
  }

  void LinkGeometryProperty::generateConvexMesh()
  {
    // create formats filter
    std::string filter;
    for(auto& p:SUPPORTED_MESH_EXTENSIONS_MAP)
    {
      filter = filter + p.first + " (*." + p.second + ");;";
    }
    filter = filter.substr(0,filter.length()-2);

    QString qpath = QFileDialog::getOpenFileName(0,QString("Generate Convex Hull from Mesh"),
                                                    QString::fromStdString(browse_start_dir_),
                                                    QString::fromStdString(filter));

    if(qpath.isEmpty())
    {
      return;
    }

  }

  void LinkGeometryProperty::loadData()
  {
    loading_ = true;
    QtVariantProperty *item;
    QString name;
    QList<QtProperty *> sub_items = top_item_->subProperties();

    // disable all
    for (int i = 0; i < sub_items.length(); ++i)
    {
      if(sub_items[i]->propertyName().toStdString().compare("Type") == 0)
      {
        continue;
      }

      item = static_cast<QtVariantProperty *>(sub_items[i]);
      item->setEnabled(false);
      top_item_->removeSubProperty(item);
    }

    // updating sub items
    sub_items = top_item_->subProperties();

    // find property func
    auto find_property = [&sub_items](const std::string& name)
    {

      for(auto& p : sub_items)
      {
        if(p->propertyName().toStdString() == name)
        {
          return static_cast<QtVariantProperty *>(p);
        }
      }
      return static_cast<QtVariantProperty *>(nullptr);
    };

    // set property func
    auto set_property_value = [this, &find_property,&sub_items](const std::string& name,  double v)
    {
      QtVariantProperty* prop = find_property(name);

      if(prop == nullptr)
      {
        prop = manager_->addProperty(QVariant::Double, tr(name.c_str()));
      }
      prop->setEnabled(true);
      prop->setValue(v);
      return prop;
    };


    // enable geometry specific subproperties
    switch(geometry_->type)
    {
      case urdf::Geometry::BOX:
      {
        boost::shared_ptr<urdf::Box> box = boost::static_pointer_cast<urdf::Box>(geometry_);

        item = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), tr("Size"));

        // create box properties
        std::vector<std::string> names = {"Length X (m)", "Length Y (m)", "Length Z (m)"};
        std::vector<double> vals = {box->dim.x, box->dim.y, box->dim.z};

        QtVariantProperty* sub_item;
        for(auto i = 0u; i < names.size() ; i++)
        {
          sub_item = set_property_value(names[i],vals[i]);
          item->addSubProperty(sub_item);
        }
        top_item_->addSubProperty(item);
      }
      break;

      case urdf::Geometry::CYLINDER:
      {
        boost::shared_ptr<urdf::Cylinder> cylinder = boost::static_pointer_cast<urdf::Cylinder>(geometry_);

        // create cylinder properties
        std::vector<std::string> names = {"Radius (m)", "Length (m)"};
        std::vector<double> vals = {cylinder->radius, cylinder->length};
        QtVariantProperty* sub_item;
        for(auto i = 0u; i < names.size() ; i++)
        {
          sub_item = set_property_value(names[i],vals[i]);
          top_item_->addSubProperty(sub_item);
        }
      }
      break;

      case urdf::Geometry::SPHERE:
      {
        boost::shared_ptr<urdf::Sphere> sphere = boost::static_pointer_cast<urdf::Sphere>(geometry_);
        QtVariantProperty* sub_item = set_property_value("Radius (m)",sphere->radius);
        top_item_->addSubProperty(sub_item);
      }
      break;

      case urdf::Geometry::MESH:
      {
        boost::shared_ptr<urdf::Mesh> mesh = boost::static_pointer_cast<urdf::Mesh>(geometry_);


        // file name
        item = find_property("File Name");
        if(item == nullptr)
        {
          item = manager_->addProperty(QVariant::String, tr("File Name"));
          top_item_->addSubProperty(item);
        }
        item->setEnabled(false);
        item->setValue(QString::fromStdString(mesh_path_));

        // set mesh properties
        item = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), tr("Scale"));
        std::vector<std::string> names = {"X", "Y", "Z"};
        std::vector<double> vals = {mesh->scale.x, mesh->scale.y,mesh->scale.z};
        QtVariantProperty* sub_item;
        for(auto i = 0u; i < names.size() ; i++)
        {
          sub_item = set_property_value(names[i],vals[i]);
          item->addSubProperty(sub_item);
        }
        top_item_->addSubProperty(item);

      }
      break;
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

      loadData();
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
}
