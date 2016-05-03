#include <urdf_editor/utils/convex_hull_generator.h>
#include <assimp/postprocess.h>
#include <ros/console.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/surface/convex_hull.h>
#include <boost/algorithm/string.hpp>
#include <string>

namespace urdf_editor
{
namespace utils
{

ConvexHullGenerator::ConvexHullGenerator():
    scene_(nullptr)
{
  output_extensions_={"dae"};
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

  if(!scene_)
  {
    ROS_ERROR_STREAM("Mesh file has not been loaded");
    return false;
  }

  auto check_extension = [this](const std::string e)
  {
    auto iter = std::find(output_extensions_.begin(),output_extensions_.end(),e);
    return iter != output_extensions_.end();
  };

  // check extensions
  std::string ext = file_path.substr(file_path.find_last_of('.')+1);
  boost::algorithm::to_lower(ext);
  if(!check_extension(ext))
  {
    ROS_ERROR("Extension %s is not supported",ext.c_str());
    return false;
  }

  // finding extension id
  int ext_count = aiGetExportFormatCount();
  std::string ext_id;
  for(auto i = 0u; i < ext_count; i++)
  {
    const aiExportFormatDesc* desc = aiGetExportFormatDescription(i);
    if(std::string(desc->fileExtension).compare(ext) == 0) // find binary
    {
      ext_id = desc->id;
      break;
    }
  }

  if(ext_id.empty())
  {
    ROS_ERROR("Assimp exporter does not support extension %s",ext.c_str());
    return false;
  }

  aiReturn res = exporter.Export(scene_,ext_id,file_path);
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
  std::size_t num_vertices = faces.size()*vertices_per_face;
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
    for(std::size_t v = 0; v < vertices.size() ; v++)
    {
      PointXYZ& p = chull_points->points[vertices[v]];
      chull_mesh_->mVertices[start_index + v] = aiVector3D(p.x,p.y,p.z);
      face.mIndices[v] = start_index + v;
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
