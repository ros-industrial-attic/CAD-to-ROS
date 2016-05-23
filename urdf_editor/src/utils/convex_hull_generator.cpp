#include <urdf_editor/utils/convex_hull_generator.h>
#include <assimp/postprocess.h>
#include <ros/console.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/surface/convex_hull.h>
#include <boost/algorithm/string.hpp>
#include <string>
#include <fstream>
#include <boost/assign.hpp>

static const std::string STL_BIN_FORMAT_ID = "stlb";
static const std::map<std::string,std::string> OUTPUT_EXTENSION_MAP = boost::assign::map_list_of ("stl","stlb");

namespace urdf_editor
{
namespace utils
{

ConvexHullGenerator::ConvexHullGenerator():
    scene_(NULL)
{

}

ConvexHullGenerator::~ConvexHullGenerator()
{

}

bool ConvexHullGenerator::generate(const std::string& file_path)
{
  using namespace Assimp;

  const aiScene* scene = importer_.ReadFile(file_path,
                                            aiProcess_Triangulate|
                                            aiProcess_JoinIdenticalVertices|
                                            aiProcess_GenNormals);

  if(!scene)
  {
    ROS_ERROR_STREAM("Read Error: " <<importer_.GetErrorString());
    return false;
  }

  if(!scene->HasMeshes())
  {
    ROS_ERROR_STREAM("No meshes were found in file "<<file_path);
    return false;
  }

  // copying scene
  aiCopyScene(scene,&scene_);

  return generateConvexHull(scene_);
}

bool ConvexHullGenerator::save(const std::string& file_path)
{
  using namespace Assimp;
  Exporter exporter;

  // register STL binary exporter
  unsigned int steps = aiProcess_Triangulate | aiProcess_SortByPType | aiProcess_GenSmoothNormals |
      aiProcess_JoinIdenticalVertices | aiProcess_FixInfacingNormals | aiProcess_FindInvalidData ;

  Exporter::ExportFormatEntry format_entry(STL_BIN_FORMAT_ID.c_str(),"Stereolithography (binary)",
                                           "stl",&assimp::STLExporter::ExportSceneSTLBinary,
                                           steps);
  if(exporter.RegisterExporter(format_entry)!= aiReturn_SUCCESS)
  {
    ROS_ERROR_STREAM("Failed to register Binary STL exporter: "<<exporter.GetErrorString());
    return false;
  }

  if(!chull_mesh_)
  {
    ROS_ERROR_STREAM("Mesh file has not been loaded");
    return false;
  }

  // check extensions
  std::string ext = file_path.substr(file_path.find_last_of('.')+1);
  boost::algorithm::to_lower(ext);
  if(OUTPUT_EXTENSION_MAP.count(ext) <= 0)
  {
    ROS_ERROR("Extension %s is not supported",ext.c_str());
    return false;
  }

  // veryfing support for selected format
  int ext_count = exporter.GetExportFormatCount();
  std::string ext_id = OUTPUT_EXTENSION_MAP.at(ext);
  bool supported = false;
  for(unsigned int i = 0u; i < ext_count; i++)
  {
    const aiExportFormatDesc* desc = exporter.GetExportFormatDescription(i);
    if(std::string(desc->id).compare(ext_id) == 0) // find binary
    {
      supported = true;
      break;
    }
  }

  if(!supported)
  {
    ROS_ERROR("Assimp exporter does not support extension %s",ext.c_str());
    return false;
  }

  aiReturn res = exporter.Export(scene_,ext_id,file_path,steps);
  if(res == aiReturn_OUTOFMEMORY)
  {
    ROS_ERROR("Mesh export failed due to out-of-memory error");
    return false;
  }

  return res == aiReturn_SUCCESS;
}

bool ConvexHullGenerator::generateConvexHull(const aiScene* scene)
{
  using namespace pcl;

  const aiMesh* mesh = scene->mMeshes[0];
  if(mesh->mNumVertices == 0)
  {
    ROS_ERROR("Mesh geometry is empty");
    return false;
  }


  PointCloud<PointXYZ>::Ptr mesh_points(new PointCloud<PointXYZ>()), chull_points(new PointCloud<PointXYZ>());
  PointXYZ p;
  for(std::size_t i = 0; i < mesh->mNumVertices; i++)
  {
    p.x = mesh->mVertices[i].x;
    p.y = mesh->mVertices[i].y;
    p.z = mesh->mVertices[i].z;
    mesh_points->points.push_back(p);
  }

  // generate chull
  std::vector< Vertices > faces;
  ConvexHull<PointXYZ> chull;
  chull.setInputCloud(mesh_points);
  chull.reconstruct(*chull_points,faces);

  // creating assimp mesh from pcl convex-hull
  std::size_t vertices_per_face = 3;
  std::size_t num_vertices = chull_points->points.size();
  chull_mesh_.reset(new aiMesh());
  chull_mesh_->mMaterialIndex = 0;
  chull_mesh_->mVertices = new aiVector3D[num_vertices];
  chull_mesh_->mNumVertices = num_vertices;
  chull_mesh_->mFaces = new aiFace[faces.size()];
  chull_mesh_->mNumFaces = faces.size();
  chull_mesh_->mName = "convex-hull";

  for(std::size_t f = 0; f < faces.size();f++)
  {
    std::vector<unsigned int>& vertices = faces[f].vertices;

    aiFace& face = chull_mesh_->mFaces[f];
    face.mIndices = new unsigned int[vertices_per_face];
    face.mNumIndices = vertices_per_face;

    std::size_t start_index = f*vertices_per_face;
    std::size_t vertex_index;
    for(std::size_t v = 0; v < vertices.size() ; v++)
    {
      vertex_index = vertices[v];
      PointXYZ& p = chull_points->points[vertex_index];
      chull_mesh_->mVertices[vertex_index] = aiVector3D(p.x,p.y,p.z);
      face.mIndices[v] = vertex_index;
    }
  }

  // deleting old scene data
  delete scene_->mRootNode;
  delete scene_->mMeshes;

  // populating scene with new mesh
  scene_->mRootNode = new aiNode();

  scene_->mMeshes = new aiMesh*[1];
  scene_->mMeshes[0] = chull_mesh_.get();
  scene_->mMeshes[ 0 ]->mMaterialIndex = 0;
  scene_->mNumMeshes = 1;

  scene_->mRootNode->mMeshes = new unsigned int[ 1 ];
  scene_->mRootNode->mMeshes[ 0 ] = 0;
  scene_->mRootNode->mNumMeshes = 1;

  return !chull_points->points.empty();

}

} /* namespace utils */
} /* namespace urdf_editor */
