#ifndef URDF_TRANSFORMS_H
#define URDF_TRANSFORMS_H

#include <ros/ros.h>
#include <tf/tfMessage.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>


#include <boost/thread/locks.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>



class URDFTransformer{

public:

  URDFTransformer();
  ~URDFTransformer();

  void updateTransforms();

  void addLink(std::string parent, std::string child);

  void removeLink(std::string name);

  void updateLink(std::string parent, std::string child);

  void updateLink(std::string parent, geometry_msgs::Quaternion quat);

  void updateLink(std::string parent, geometry_msgs::Vector3 origin);

  void updateLink(std::string parent, geometry_msgs::Transform trans);

private:

  tf::tfMessage frames_;

  boost::thread *worker_;
  boost::mutex data_lock_;

  tf::TransformBroadcaster broadcaster_;

  void worker_thread();

};

#endif // URDF_TRANSFORMS_H
