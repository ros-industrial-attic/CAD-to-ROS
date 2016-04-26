
#include <qttreepropertybrowser.h>
#include <qtvariantproperty.h>

#include <urdf_editor/common.h>

#include <QTranslator>

#include <urdf_model/pose.h>


namespace urdf_editor
{
  QString Common::attributeStr(int enumValue)
  {
    switch (enumValue)
    {
    case urdf_editor::Constraint:
      return QString("constraint");
    case urdf_editor::SingleStep:
      return QString("singleStep");
    case urdf_editor::Decimals:
      return QString("decimals");
    case urdf_editor::EnumIcons:
      return QString("enumIcons");
    case urdf_editor::FlagNames:
      return QString("flagNames");
    case urdf_editor::Maximum:
      return QString("maximum");
    case urdf_editor::Minimum:
      return QString("minimum");
    case urdf_editor::RegExp:
      return QString("regExp");
    case urdf_editor::EchoMode:
      return QString("echoMode");
    case urdf_editor::ReadOnly:
      return QString("readOnly");
    case urdf_editor::TextVisible:
      return QString("textVisible");
    case urdf_editor::EnumNames:
      return QString("enumNames");
    default:
      return QString("");
    }
  }

  void Common::addOriginProperty(QtVariantPropertyManager *manager, QtProperty *top_item, urdf::Pose &orgin)
  {
    QtVariantProperty *item;
    QtVariantProperty *sub_item;
    double r, p, y;

    orgin.rotation.getRPY(r, p, y);

    QtProperty *origin = manager->addProperty(QtVariantPropertyManager::groupTypeId(), QTranslator::tr("Origin (optional)"));
    top_item->addSubProperty(origin);

    // Create position properties
    item = manager->addProperty(QtVariantPropertyManager::groupTypeId(), QTranslator::tr("Position (m)"));
    sub_item = manager->addProperty(QVariant::Double, QTranslator::tr("X"));
    sub_item->setValue(orgin.position.x);
    sub_item->setAttribute(attributeStr(Decimals), 6);
    item->addSubProperty(sub_item);
    sub_item = manager->addProperty(QVariant::Double, QTranslator::tr("Y"));
    sub_item->setValue(orgin.position.y);
    sub_item->setAttribute(attributeStr(Decimals), 6);
    item->addSubProperty(sub_item);
    sub_item = manager->addProperty(QVariant::Double, QTranslator::tr("Z"));
    sub_item->setValue(orgin.position.z);
    sub_item->setAttribute(attributeStr(Decimals), 6);
    item->addSubProperty(sub_item);
    origin->addSubProperty(item);

    // Create orientation properties
    item = manager->addProperty(QtVariantPropertyManager::groupTypeId(), QTranslator::tr("Orientation (rad)"));
    sub_item = manager->addProperty(QVariant::Double, QTranslator::tr("Roll"));
    sub_item->setValue(r);
    sub_item->setAttribute(attributeStr(Decimals), 6);
    item->addSubProperty(sub_item);
    sub_item = manager->addProperty(QVariant::Double, QTranslator::tr("Pitch"));
    sub_item->setValue(p);
    sub_item->setAttribute(attributeStr(Decimals), 6);
    item->addSubProperty(sub_item);
    sub_item = manager->addProperty(QVariant::Double, QTranslator::tr("Yaw"));
    sub_item->setValue(y);
    sub_item->setAttribute(attributeStr(Decimals), 6);
    item->addSubProperty(sub_item);
    origin->addSubProperty(item);
  }
}
