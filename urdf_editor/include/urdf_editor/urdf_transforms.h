#ifndef URDF_TRANSFORMS_H
#define URDF_TRANSFORMS_H

#include <ros/ros.h>
#include <tf/tfMessage.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>


#include <boost/thread/locks.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>


namespace urdf_editor
{
class URDFTransformer{

public:

  URDFTransformer();
  ~URDFTransformer();

  /**
   * @brief addLink Adds a link to the transfrom list to be published
   * @param parent The name of the parent frame id for the link
   * @param child The name of the child frame id for the link
   */
  void addLink(std::string parent, std::string child);

  /**
   * @brief removeLink Removes a link from the list if it exists
   * @param name The name of the link to remove from the list
   */
  void removeLink(std::string name);

  /**
   * @brief updateLink Looks for a link in the existing tree to update the child information for
   * @param parent The name of the link to find and update
   * @param child The name of the child to update to
   */
  void updateLink(std::string parent, std::string child);

  /**
   * @brief updateLink Looks for a link in the existing tree to update the origin orientation information for
   * @param parent The name of the link to find and update
   * @param quat The orientation, in quaternion format
   */
  void updateLink(std::string parent, geometry_msgs::Quaternion quat);

  /**
   * @brief updateLink Looks for a link in the existing tree to update the origin position information for
   * @param parent The name of the link to find and update
   * @param origin The origin position
   */
  void updateLink(std::string parent, geometry_msgs::Vector3 origin);

  /**
   * @brief updateLink Looks for a link in the existing tree to update the origin information for
   * @param parent The name of the link to find and update
   * @param trans The origin information, in tranformation format
   */
  void updateLink(std::string parent, geometry_msgs::Transform trans);

private:

  tf::tfMessage frames_; /**< @brief The list of frames to be published for visualization */

  boost::thread *worker_;  /**< @brief The worker thread, created on construction, for publishing the TF frames */
  boost::mutex data_lock_; /**< @brief The mutex data lock for reading/writing the TF frames */

  tf::TransformBroadcaster broadcaster_; /**< @brief The TF broadcaster for publishing frames to the /tf ROS topic */

  void worker_thread();

  bool working;

};

}
#endif // URDF_TRANSFORMS_H
