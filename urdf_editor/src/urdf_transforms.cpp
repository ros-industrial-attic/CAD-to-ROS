#include <urdf_editor/urdf_transforms.h>



URDFTransformer::URDFTransformer()
{
  worker_ = new boost::thread(boost::bind(&URDFTransformer::worker_thread, this));
}

URDFTransformer::~URDFTransformer()
{
  delete worker_;
}

void URDFTransformer::worker_thread()
{
  ros::Rate rt = 10;
  while(ros::ok())
  {
    tf::tfMessage tf_copy;
    {
      boost::lock_guard<boost::mutex> lock(data_lock_);
      tf_copy = frames_;
    }

    if(tf_copy.transforms.size() > 0)
    {
      broadcaster_.sendTransform(tf_copy.transforms);
    }
    rt.sleep();
  }
}

void URDFTransformer::addLink(std::string parent, std::string child)
{
  boost::lock_guard<boost::mutex> lock(data_lock_);
  for(int i = 0; i < frames_.transforms.size(); ++i)
  {
    if(parent.compare(frames_.transforms[i].header.frame_id) == 0)
    {
      ROS_WARN("frame name %s already exists, not adding to list", parent.c_str());
      return;
    }
  }

  geometry_msgs::TransformStamped new_frame;
  new_frame.header.frame_id = parent;
  new_frame.child_frame_id = child;
  frames_.transforms.push_back(new_frame);
}

void URDFTransformer::updateLink(std::string parent, std::string child)
{
  boost::lock_guard<boost::mutex> lock(data_lock_);
  int i;
  for(i = 0; i < frames_.transforms.size(); ++i)
  {
    // find frame to update
    if(parent.compare(frames_.transforms[i].header.frame_id) == 0)
    {
      frames_.transforms[i].child_frame_id = child;
      break;
    }
  }

  // Do we need to add the transform here or just return a bool success/failure?
  if(i == frames_.transforms.size())
  {
    geometry_msgs::TransformStamped new_frame;
    new_frame.header.frame_id = parent;
    new_frame.child_frame_id = child;
    frames_.transforms.push_back(new_frame);
  }
}

void URDFTransformer::updateLink(std::string parent, geometry_msgs::Transform trans)
{
  updateLink(parent, trans.rotation);
  updateLink(parent, trans.translation);
}

void URDFTransformer::updateLink(std::string parent, geometry_msgs::Quaternion quat)
{
  boost::lock_guard<boost::mutex> lock(data_lock_);
  for(int i = 0; i < frames_.transforms.size(); ++i)
  {
    // find frame to update
    if(parent.compare(frames_.transforms[i].header.frame_id) == 0)
    {
      frames_.transforms[i].transform.rotation = quat;
      break;
    }
  }
}

void URDFTransformer::updateLink(std::string parent, geometry_msgs::Vector3 origin)
{
  boost::lock_guard<boost::mutex> lock(data_lock_);
  for(int i = 0; i < frames_.transforms.size(); ++i)
  {
    // find frame to update
    if(parent.compare(frames_.transforms[i].header.frame_id) == 0)
    {
      frames_.transforms[i].transform.translation = origin;
      break;
    }
  }
}

