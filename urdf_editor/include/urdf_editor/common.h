#ifndef __URDF_EDITOR_COMMON_H__
#define __URDF_EDITOR_COMMON_H__

#include <urdf_editor/qt_types.h>
#include <QString>

#include <urdf_editor/urdf_types_ext.h>


namespace urdf_editor
{
  enum Attributes {Constraint, SingleStep, Decimals, EnumIcons, FlagNames, Maximum, Minimum, RegExp, EchoMode, ReadOnly, TextVisible, EnumNames};
  class Common
  {
  public:
    static QString attributeStr(int enumValue);
    static void addOriginProperty(QtVariantPropertyManager *manager, QtProperty *top_item, urdf::Pose &orgin);
  };

}

typedef urdf_editor::Attributes Attributes;

#endif // __URDF_EDITOR_COMMON_H__
