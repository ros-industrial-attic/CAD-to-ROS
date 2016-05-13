#ifndef CAD_TO_ROS_STEP_TO_STL_H
#define CAD_TO_ROS_STEP_TO_STL_H

#include <string>

namespace urdf_editor
{
namespace codecs
{

bool convertStepToStl(const std::string& step_file_path, const std::string& stl_file_path);

}

}

#endif
