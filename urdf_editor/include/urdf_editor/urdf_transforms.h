#ifndef __URDF_TRANSFORMS_H__
#define __URDF_TRANSFORMS_H__


#include <urdf_editor/property_types.h>

#include <tf/tfMessage.h>
#include <tf/transform_broadcaster.h>

#include <boost/thread/mutex.hpp>

#include <string>


namespace boost
{
  class thread;
}


namespace urdf_editor
{
class URDFTransformer
{

public:

  URDFTransformer();
  ~URDFTransformer();

  /**
   * @brief addLink Adds a link to the transfrom list to be published
   * @param parent The name of the parent frame id for the link
   * @param child The name of the child frame id for the link
   */
  void addLink(const std::string parent, const std::string child);

  void clear();

  /**
   * @brief removeLink Removes a link from the list if it exists
   * @param name The name of the link to remove from the list
   */
  void removeLink(const std::string parent, const std::string child);

  /**
   * @brief updateLink Looks for a link in the existing tree to update the child information for
   * @param parent The name of the link to find and update
   * @param child The name of the child to find and update
   * @param new_parent The new parent link name
   * @param new_child The new child link name
   */
  void updateLink(const std::string parent, const std::string child, const std::string new_parent, const std::string new_child);

  /**
   * @brief updateLink Looks for a link in the existing tree to update the origin orientation information for
   * @param parent The name of the link to find and update
   * @param quat The orientation, in quaternion format
   */
  void updateLink(const std::string parent, const std::string child, const geometry_msgs::Quaternion quat);

  /**
   * @brief updateLink Looks for a link in the existing tree to update the origin position information for
   * @param parent The name of the link to find and update
   * @param origin The origin position
   */
  void updateLink(const std::string parent, const std::string child, const geometry_msgs::Vector3 origin);

  /**
   * @brief updateLink Looks for a link in the existing tree to update the origin information for
   * @param parent The name of the link to find and update
   * @param trans The origin information, in tranformation format
   */
  void updateLink(const std::string parent, const std::string child, const geometry_msgs::Transform trans);

  void updateLink(JointProperty *property);

private:

  tf::tfMessage frames_; /**< @brief The list of frames to be published for visualization */

  boost::thread *worker_;  /**< @brief The worker thread, created on construction, for publishing the TF frames */
  boost::mutex data_lock_; /**< @brief The mutex data lock for reading/writing the TF frames */

  tf::TransformBroadcaster broadcaster_; /**< @brief The TF broadcaster for publishing frames to the /tf ROS topic */

  void worker_thread();

  int findLink(const std::string parent, const std::string child);

  bool working;

};

}
#endif // __URDF_TRANSFORMS_H__
