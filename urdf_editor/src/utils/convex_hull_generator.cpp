#include <urdf_editor/utils/convex_hull_generator.h>
#include <assimp/postprocess.h>
#include <ros/console.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/surface/convex_hull.h>
#include <string>


namespace urdf_editor
{
namespace utils
{

ConvexHullGenerator::ConvexHullGenerator()
{
  // TODO Auto-generated constructor stub

}

ConvexHullGenerator::~ConvexHullGenerator()
{
  // TODO Auto-generated destructor stub
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
    ROS_ERROR_STREAM(importer_.GetErrorString());
    return false;
  }

  if(!scene->HasMeshes())
  {
    ROS_ERROR_STREAM("No meshes were found in file "<<file_path);
    return false;
  }

  return generateConvexHull(scene);
}

bool ConvexHullGenerator::save(const std::string& file_path)
{
  using namespace Assimp;
  Exporter exporter;

  auto check_extension = [this](const std::string e)
  {
    auto iter = std::find(supported_extensions_.begin(),supported_extensions_.end(),e);
    return iter != supported_extensions_.end();
  };

  // check extensions
  std::string ext = file_path.substr(file_path.find_last_of('.')+1);
  if(!check_extension(ext))
  {
    ROS_ERROR("Extension %s is not supported",ext.c_str());
    return false;
  }

  // creating scene
  boost::shared_ptr<aiScene> scene;
  scene.reset(new aiScene());
  scene->mRootNode = new aiNode();

  scene->mMaterials = new aiMaterial*[ 1 ];
  scene->mMaterials[ 0 ] = nullptr;
  scene->mNumMaterials = 1;
  scene->mMaterials[ 0 ] = new aiMaterial();

  scene->mMeshes = new aiMesh*[1];
  scene->mMeshes[0] = chull_mesh_.get();
  scene->mMeshes[ 0 ]->mMaterialIndex = 0;
  scene->mNumMeshes = 1;

  scene->mRootNode->mMeshes = new unsigned int[ 1 ];
  scene->mRootNode->mMeshes[ 0 ] = 0;
  scene->mRootNode->mNumMeshes = 1;


  aiReturn res = exporter.Export(scene.get(),"",file_path);
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

  return !chull_points->points.empty();

}

} /* namespace utils */
} /* namespace urdf_editor */
