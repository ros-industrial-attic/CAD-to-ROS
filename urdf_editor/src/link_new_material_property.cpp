
#include <qttreepropertybrowser.h>
#include <qtvariantproperty.h>

#include <urdf_editor/link_new_material_property.h>
#include <urdf_editor/common.h>

#include <urdf_model/link.h>


namespace urdf_editor
{
  LinkNewMaterialProperty::LinkNewMaterialProperty(urdf::MaterialSharedPtr material): material_(material), manager_(new QtVariantPropertyManager()), factory_(new QtVariantEditorFactory())
  {
    loading_ = true;
    QtVariantProperty *item;

    QObject::connect(manager_, SIGNAL(valueChanged(QtProperty *, const QVariant &)),
              this, SLOT(onValueChanged(QtProperty *, const QVariant &)));

    top_item_ = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), tr("Material"));

    item = manager_->addProperty(QVariant::String, tr("Name"));
    top_item_->addSubProperty(item);

    item = manager_->addProperty(QVariant::Color, tr("Color"));
    top_item_->addSubProperty(item);

    item = manager_->addProperty(QVariant::String, tr("Texture"));
    top_item_->addSubProperty(item);

    loading_ = false;
  }

  LinkNewMaterialProperty::~LinkNewMaterialProperty()
  {
    delete manager_;
    delete factory_;
  }

  void LinkNewMaterialProperty::loadData()
  {
    loading_ = true;
    QtVariantProperty *item;
    QString name;
    QList<QtProperty *> sub_items = top_item_->subProperties();
    for (int i = 0; i < sub_items.length(); ++i)
    {
      item = static_cast<QtVariantProperty *>(sub_items[i]);
      name = item->propertyName();

      if (name == "Name")
        item->setValue(QString::fromStdString(material_->name));
      else if (name == "Color")
        item->setValue(QColor::fromRgba(qRgba(material_->color.r * 255.0, material_->color.g * 255.0, material_->color.b * 255.0, material_->color.a * 255.0)));
      else if (name == "Texture")
        item->setValue(QString::fromStdString(material_->texture_filename));
    }
    loading_ = false;
  }

  void LinkNewMaterialProperty::loadFactoryForManager(QtTreePropertyBrowserSharedPtr& property_editor)
  {
    property_editor->setFactoryForManager(manager_, factory_);
  }

  void LinkNewMaterialProperty::onValueChanged(QtProperty *property, const QVariant &val)
  {
    if (loading_)
      return;

    QString name = property->propertyName();
    if (name == "Name")
      material_->name = val.toString().toStdString();
    else if (name == "Red")
      material_->color.r = val.toDouble()/255.0;
    else if (name == "Green")
      material_->color.g = val.toDouble()/255.0;
    else if (name == "Blue")
      material_->color.b = val.toDouble()/255.0;
    else if (name == "Alpha")
      material_->color.a = val.toDouble()/255.0;
    else if (name == "Texture")
      material_->texture_filename = val.toString().toStdString();

    emit LinkNewMaterialProperty::valueChanged(property, val);
  }
}
