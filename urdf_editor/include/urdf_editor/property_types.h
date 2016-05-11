/**
 * Forward declared property classes (and their shared pointer type).
 *
 * Author: G.A. vd. Hoorn (TU Delft Robotics Institute)
 */
#ifndef __URDF_EDITOR_PROPERTY_TYPES_H__
#define __URDF_EDITOR_PROPERTY_TYPES_H__

#include <boost/shared_ptr.hpp>

#define UE_PROPERTY_TYPEDEF_CLASS_POINTER(Class) \
class Class; \
typedef boost::shared_ptr<Class> Class##SharedPtr;


namespace urdf_editor
{

UE_PROPERTY_TYPEDEF_CLASS_POINTER(JointAxisProperty);
UE_PROPERTY_TYPEDEF_CLASS_POINTER(JointCalibrationProperty);
UE_PROPERTY_TYPEDEF_CLASS_POINTER(JointDynamicsProperty);
UE_PROPERTY_TYPEDEF_CLASS_POINTER(JointLimitsProperty);
UE_PROPERTY_TYPEDEF_CLASS_POINTER(JointMimicProperty);
UE_PROPERTY_TYPEDEF_CLASS_POINTER(JointSafetyProperty);
UE_PROPERTY_TYPEDEF_CLASS_POINTER(OriginProperty);
UE_PROPERTY_TYPEDEF_CLASS_POINTER(JointProperty);

UE_PROPERTY_TYPEDEF_CLASS_POINTER(LinkCollisionProperty);
UE_PROPERTY_TYPEDEF_CLASS_POINTER(LinkGeometryProperty);
UE_PROPERTY_TYPEDEF_CLASS_POINTER(LinkInertialProperty);
UE_PROPERTY_TYPEDEF_CLASS_POINTER(LinkNewMaterialProperty);
UE_PROPERTY_TYPEDEF_CLASS_POINTER(LinkVisualProperty);
UE_PROPERTY_TYPEDEF_CLASS_POINTER(LinkProperty);

UE_PROPERTY_TYPEDEF_CLASS_POINTER(URDFProperty);
UE_PROPERTY_TYPEDEF_CLASS_POINTER(URDFPropertyTreeLinkItem);
UE_PROPERTY_TYPEDEF_CLASS_POINTER(URDFPropertyTreeJointItem);

}

#endif // __URDF_EDITOR_PROPERTY_TYPES_H__
