/**
 * Additional forward declared urdf classes (and their shared pointer type).
 *
 * An extension to the types already declared in 'urdf_types.h'.
 *
 * Author: G.A. vd. Hoorn (TU Delft Robotics Institute)
 */
#ifndef __URDF_MODEL_TYPES_EXT_H__
#define __URDF_MODEL_TYPES_EXT_H__

#include <urdf_editor/urdf_types.h>


namespace urdf
{

URDF_TYPEDEF_CLASS_POINTER(ModelInterface);
URDF_TYPEDEF_CLASS_POINTER(Pose);
URDF_TYPEDEF_CLASS_POINTER(Vector3);

}

#endif // __URDF_MODEL_TYPES_EXT_H__
