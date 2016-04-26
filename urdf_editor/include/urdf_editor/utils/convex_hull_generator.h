
#ifndef URDF_EDITOR_UTILS_CONVEX_HULL_GENERATOR_H_
#define URDF_EDITOR_UTILS_CONVEX_HULL_GENERATOR_H_

#include <assimp/Importer.hpp>
#include <assimp/Exporter.hpp>
#include <assimp/scene.h>
#include <boost/shared_ptr.hpp>

namespace urdf_editor
{
namespace utils
{

class ConvexHullGenerator
{
public:
  ConvexHullGenerator();
  virtual ~ConvexHullGenerator();
  /**
   * @brief Generates a convex hull from the mesh file located in 'file_path'
   *
   * @param file_path   Path to the mesh file
   * @return            True when succeeded, false otherwise.
   */
  bool generate(const std::string& file_path);

  /*
   * @brief Saves the convex hull mesh in the location indicated by 'file_path'.  The
   *        convex hull must have been previously generated with the 'generate()' method.
   *
   * @param file_path   Path where the convex hull mesh file will be saved.
   * @return            True when succeeded, false otherwise.
   */
  bool save(const std::string& file_path);

protected:

  bool generateConvexHull(const aiScene* scene);

protected:

  Assimp::Importer importer_;
  boost::shared_ptr<aiMesh> chull_mesh_;
  std::vector<std::string> supported_extensions_;



};

} /* namespace utils */
} /* namespace urdf_editor */

#endif /* URDF_EDITOR_UTILS_CONVEX_HULL_GENERATOR_H_ */
