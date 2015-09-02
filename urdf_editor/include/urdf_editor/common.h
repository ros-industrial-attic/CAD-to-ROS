#ifndef COMMON_H
#define COMMON_H
#include <QtCore>
#include "urdf_editor/qtpropertybrowser/qtvariantproperty.h"
#include <urdf_model/joint.h>

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
#endif // COMMON_H
