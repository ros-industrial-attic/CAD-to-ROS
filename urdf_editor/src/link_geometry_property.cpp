
#include <qttreepropertybrowser.h>
#include <qtvariantproperty.h>
#include <qmenu.h>
#include <qaction.h>
#include <qfiledialog.h>

#include <urdf_editor/link_geometry_property.h>
#include <urdf_editor/common.h>
#include <urdf_model/link.h>
#include <urdf_editor/utils/convex_hull_generator.h>

#include <ros/package.h>
#include <ros/console.h>

static const double DEFAULT_SIDE_LENGHT = 0.1;
static const std::string PACKAGE_NAME = "urdf_builder";
static const std::string CONVEX_MESH_FILE_FILTER = "STL, COLLADA, Wavefront (*.stl *.dae *.obj)";
static const std::string LOAD_MESH_FILE_FILTER = "STL, COLLADA(*.stl *.dae )";
static const std::string OUTPUT_CHULL_EXT = "stl";

struct FilePaths
{
  std::string file_name;
  std::string ros_formatted;
  std::string parent_dir;
  std::string full_path;
  std::string ros_pkg;
  std::string local_path;
};

static FilePaths getPathFromROSFormatted(const std::string& ros_formatted_path)
{
  FilePaths paths;

  // get package
  std::string pkg = ros_formatted_path.substr(std::string("package://").size());
  pkg = pkg.substr(0,pkg.find_first_of('/')+1);

  // get file name
  std::size_t pos = ros_formatted_path.find_last_of('/');
  std::string filename = ros_formatted_path.substr(pos);

  // get local path
  std::string local_path = ros_formatted_path.substr(std::string("package://").size());
  local_path = local_path.substr(0,pos);

  // create full path
  paths.full_path = ros::package::getPath(pkg) + "/" + local_path + "/" + filename;
  paths.file_name = filename;
  paths.local_path = local_path;
  paths.ros_formatted = ros_formatted_path;
  paths.ros_pkg = pkg;


  ROS_DEBUG("full path %s",paths.full_path.c_str());
  ROS_DEBUG("file_name %s",paths.file_name.c_str());
  ROS_DEBUG("local path %s",paths.local_path.c_str());
  ROS_DEBUG("ros path %s",paths.ros_formatted.c_str());
  ROS_DEBUG("ros package %s",paths.ros_pkg.c_str());

  return paths;
}


static FilePaths getROSFormattedPath(const std::string& full_path)
{

  FilePaths paths;

  // determining file and path
  std::string pkg,local_path;
  std::size_t file_pos = full_path.find_last_of("/");
  std::string file_name = full_path.substr(file_pos+1);
  paths.full_path = full_path;
  paths.file_name = file_name;

  // finding ros package
  std::size_t start_pos, end_pos = file_pos;
  std::string partial_path = full_path.substr(0,end_pos);
  paths.parent_dir = partial_path;
  while(start_pos != std::string::npos)
  {
    start_pos = partial_path.find_last_of("/");
    if(start_pos ==  std::string::npos )
    {
      ROS_ERROR("The file %s is not located inside a ros package",file_name.c_str());
      return paths;
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

  paths.ros_formatted = "package://"+ pkg + "/" + local_path +  file_name;
  paths.ros_pkg = pkg;
  paths.local_path = local_path;
  return paths;
}

namespace urdf_editor
{
  LinkGeometryProperty::LinkGeometryProperty(urdf::GeometrySharedPtr geometry):
      geometry_(geometry),
      manager_(new QtVariantPropertyManager()),
      factory_(new QtVariantEditorFactory()),
      browse_start_dir_(ros::package::getPath(PACKAGE_NAME))
  {
    loading_ = true;

    // creating default geometries
    urdf::SphereSharedPtr sphere(new urdf::Sphere());
    urdf::BoxSharedPtr box(new urdf::Box());
    urdf::CylinderSharedPtr cylinder(new urdf::Cylinder());
    urdf::MeshSharedPtr mesh(new urdf::Mesh());
    sphere->radius = DEFAULT_SIDE_LENGHT;
    box->dim.x = box->dim.y = box->dim.z = DEFAULT_SIDE_LENGHT;
    cylinder->length = DEFAULT_SIDE_LENGHT;
    cylinder->radius = 0.5*DEFAULT_SIDE_LENGHT;
    mesh->scale = urdf::Vector3(1,1,1);

    geometries_map_ = {{int(urdf::Geometry::SPHERE),sphere},
                       {int(urdf::Geometry::BOX),box},
                       {int(urdf::Geometry::CYLINDER),cylinder},
                       {int(urdf::Geometry::MESH),mesh}};

    geometries_map_[int(geometry_->type)] = geometry_;

    QObject::connect(manager_, SIGNAL(valueChanged(QtProperty *, const QVariant &)),
              this, SLOT(onValueChanged(QtProperty *, const QVariant &)));
    //{SPHERE, BOX, CYLINDER, MESH}
    top_item_ = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), tr("Geometry"));
    type_item_ = manager_->addProperty(QtVariantPropertyManager::enumTypeId(), tr("Type"));
    type_item_->setAttribute(Common::attributeStr(EnumNames), QStringList() << "SPHERE" << "BOX" << "CYLINDER" << "MESH");
    top_item_->addSubProperty(type_item_);
    type_item_->setValue(geometry_->type);

    loading_ = false;
    loadData();

  }

  LinkGeometryProperty::~LinkGeometryProperty()
  {
    delete manager_;
    delete factory_;
  }

  void LinkGeometryProperty::loadMesh()
  {
    // open file dialog
    QString qpath = QFileDialog::getOpenFileName(0,QString("Open Mesh"),
                                                   QString::fromStdString(browse_start_dir_),
                                                   QString::fromStdString(LOAD_MESH_FILE_FILTER));

    if(qpath.isEmpty())
    {
      return;
    }

    FilePaths paths = getROSFormattedPath(qpath.toStdString());
    if(paths.ros_formatted.empty())
    {
      return;
    }

    browse_start_dir_ = paths.parent_dir;
    ROS_INFO_STREAM("ROS mesh path "<<paths.ros_formatted);

    // updating mesh info
    boost::shared_ptr<urdf::Mesh> mesh = boost::static_pointer_cast<urdf::Mesh>(
        geometries_map_[int(urdf::Geometry::MESH)]);
    mesh->scale.x = mesh->scale.y = mesh->scale.z = 1.0;
    mesh->filename = paths.ros_formatted;
    geometry_ = mesh;
    geometry_->type = urdf::Geometry::MESH;
    type_item_->setValue(urdf::Geometry::MESH);

    loadData();

  }

  bool LinkGeometryProperty::generateConvexMesh()
  {
    // create formats filter
    QString qpath = QFileDialog::getOpenFileName(0,QString("Generate Convex Hull from Mesh"),
                                                    QString::fromStdString(browse_start_dir_),
                                                    QString::fromStdString(CONVEX_MESH_FILE_FILTER));
    if(qpath.isEmpty())
    {
      return false;
    }
    FilePaths paths = getROSFormattedPath(qpath.toStdString());
    if(paths.ros_formatted.empty())
    {
      return false;
    }

    // getting currently designated mesh path
/*    boost::shared_ptr<urdf::Mesh> mesh = boost::static_pointer_cast<urdf::Mesh>(
        geometries_map_[int(urdf::Geometry::MESH)]);
    mesh->filename;*/

    browse_start_dir_ = paths.parent_dir;

    // generate convex full and save mesh
    utils::ConvexHullGenerator chull_gen;
    std::string chull_file = "chull-" + paths.file_name;
    chull_file = chull_file.substr(0,chull_file.find_last_of('.')+1) + OUTPUT_CHULL_EXT;
    std::string chull_path = paths.parent_dir + "/" + chull_file;
    if(chull_gen.generate(paths.full_path))
    {
      if(!chull_gen.save(chull_path))
      {
        ROS_ERROR_STREAM("Failed to save convex hull mesh file at location"<<chull_path);
        return false;
      }
    }
    else
    {
      ROS_ERROR_STREAM("Failed to generate convex hull from file "<<paths.full_path);
      return false;
    }

    // updating mesh info
    boost::shared_ptr<urdf::Mesh> mesh = boost::static_pointer_cast<urdf::Mesh>(
        geometries_map_[int(urdf::Geometry::MESH)]);
    mesh->scale.x = mesh->scale.y = mesh->scale.z = 1.0;
    mesh->filename = "package://"+ paths.ros_pkg + "/" + paths.local_path +  chull_file;
    geometry_ = mesh;
    geometry_->type = urdf::Geometry::MESH;
    type_item_->setValue(urdf::Geometry::MESH);

    getPathFromROSFormatted(mesh->filename);

    loadData();

    return true;

  }

  void LinkGeometryProperty::loadData()
  {
    loading_ = true;
    QtVariantProperty *item;
    QString name;
    QList<QtProperty *> sub_items = top_item_->subProperties();

    // remove all
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

    // updating sub items list
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


    // create geometry specific sub-properties
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
        item->setValue(QString::fromStdString(mesh->filename));

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
    emit LinkGeometryProperty::valueChanged(static_cast<QtProperty* >(type_item_),
                                            QVariant::fromValue(int(geometry_->type)));
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
    int type = val.toInt();
    if (name == "Type")
    {

      int type = val.toInt();
      if(geometries_map_.count(type) == 0)
      {
        return;
      }

      geometry_ = geometries_map_[type];

      switch (type) //{SPHERE, BOX, CYLINDER, MESH}
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

      emit LinkGeometryProperty::valueChanged(property, val);
    }

  }
}
