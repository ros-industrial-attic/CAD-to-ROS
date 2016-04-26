
#include <urdf_editor/urdf_transforms.h>

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>

#include <boost/thread/locks.hpp>
#include <boost/thread.hpp>

#include <urdf_editor/joint_property.h>
#include <urdf_editor/origin_property.h>

#include <urdf_model/pose.h>


namespace urdf_editor {

URDFTransformer::URDFTransformer()
{
  worker_ = new boost::thread(boost::bind(&URDFTransformer::worker_thread, this));
}

URDFTransformer::~URDFTransformer()
{
  working_ = false;
  worker_->join();
  delete worker_;
}

void URDFTransformer::worker_thread()
{
  ros::Rate rt = 10;
  working_ = true;
  while(ros::ok() && working_)
  {
    tf::tfMessage tf_copy;
    {
      boost::lock_guard<boost::mutex> lock(data_lock_);
      tf_copy = frames_;
    }

    if(tf_copy.transforms.size() > 0)
    {
      for(int i = 0; i < tf_copy.transforms.size(); ++i)
      {
        tf_copy.transforms[i].header.stamp = ros::Time::now();
      }
      broadcaster_.sendTransform(tf_copy.transforms);
    }
    rt.sleep();
  }
}

void URDFTransformer::addLink(const std::string parent, const std::string child)
{
  boost::lock_guard<boost::mutex> lock(data_lock_);
  int index = findLink(parent, child);
  if (index >= 0 && index < frames_.transforms.size())
  {
    geometry_msgs::TransformStamped new_frame;
    new_frame.header.frame_id = parent;
    new_frame.child_frame_id = child;
    frames_.transforms.push_back(new_frame);
  }
  else
  {
    ROS_WARN("frame name %s already exists, not adding to list", parent.c_str());
  }
}

void URDFTransformer::clear()
{
  boost::lock_guard<boost::mutex> lock(data_lock_);
  frames_.transforms.clear();
}

int URDFTransformer::findLink(const std::string parent, const std::string child)
{
  for(int i = 0; i < frames_.transforms.size(); ++i)
  {
    if(parent.compare(frames_.transforms[i].header.frame_id) == 0 && child.compare(frames_.transforms[i].child_frame_id) == 0)
    {
      return i;
    }
  }

  return -1;
}

void URDFTransformer::removeLink(const std::string parent, const std::string child)
{
  boost::lock_guard<boost::mutex> lock(data_lock_);
  int index = findLink(parent, child);
  if (index >= 0 && index < frames_.transforms.size())
  {
    frames_.transforms.erase(frames_.transforms.begin()+index);
  }
}

void URDFTransformer::updateLink(const std::string parent, const std::string child, const std::string new_parent, const std::string new_child)
{
  boost::lock_guard<boost::mutex> lock(data_lock_);
  int index = findLink(parent, child);
  if (index >= 0 && index < frames_.transforms.size())
  {
    frames_.transforms[index].header.frame_id = new_parent;
    frames_.transforms[index].child_frame_id = new_child;
  }
  else
  {
    geometry_msgs::TransformStamped new_frame;
    new_frame.header.frame_id = parent;
    new_frame.child_frame_id = child;
    frames_.transforms.push_back(new_frame);
  }
}

void URDFTransformer::updateLink(const std::string parent, const std::string child, const geometry_msgs::Transform trans)
{
  updateLink(parent, child, trans.rotation);
  updateLink(parent, child, trans.translation);
}

void URDFTransformer::updateLink(const std::string parent, const std::string child, const geometry_msgs::Quaternion quat)
{
  boost::lock_guard<boost::mutex> lock(data_lock_);
  int index = findLink(parent, child);

  if (index >= 0 && index < frames_.transforms.size())
  {
    frames_.transforms[index].transform.rotation = quat;
  }
  else
  {
    geometry_msgs::TransformStamped new_frame;
    new_frame.transform.rotation = quat;
    new_frame.header.frame_id = parent;
    new_frame.child_frame_id = child;
    frames_.transforms.push_back(new_frame);
  }
}

void URDFTransformer::updateLink(const std::string parent, const std::string child, const geometry_msgs::Vector3 origin)
{
  boost::lock_guard<boost::mutex> lock(data_lock_);
  int index = findLink(parent, child);

  if (index >= 0 && index < frames_.transforms.size())
  {
    frames_.transforms[index].transform.translation = origin;
  }
  else
  {
    geometry_msgs::TransformStamped new_frame;
    new_frame.transform.translation = origin;
    new_frame.header.frame_id = parent;
    new_frame.child_frame_id = child;
    frames_.transforms.push_back(new_frame);
  }
}

void URDFTransformer::updateLink(JointProperty *property)
{
  if(property->hasOriginProperty())
  {
    OriginPropertySharedPtr origin = property->getOriginProperty();
    urdf::Pose pose = origin->getOriginData();
    geometry_msgs::Vector3 vect;
    vect.x = pose.position.x;
    vect.y = pose.position.y;
    vect.z = pose.position.z;
    updateLink(property->getParent(), property->getChild(), vect);

    // Update rotation
    geometry_msgs::Quaternion quat;
    pose.rotation.getQuaternion(quat.x, quat.y, quat.z, quat.w);
    updateLink(property->getParent(), property->getChild(), quat);
  }
  else
  {
    geometry_msgs::Vector3 vect;
    updateLink(property->getParent(), property->getChild(), vect);
    geometry_msgs::Quaternion quat;
    updateLink(property->getParent(), property->getChild(), quat);
    ROS_WARN_STREAM("Joint for link " << property->getParent() << " to " << property->getChild() << " does not have origin property information.  Setting to zero.");

  }
}

}
