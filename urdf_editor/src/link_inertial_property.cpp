
#include <qttreepropertybrowser.h>
#include <qtvariantproperty.h>

#include <urdf_editor/link_inertial_property.h>
#include <urdf_editor/origin_property.h>
#include <urdf_editor/common.h>

#include <urdf_model/link.h>


namespace urdf_editor
{
  LinkInertialProperty::LinkInertialProperty(urdf::InertialSharedPtr inertial): inertial_(inertial), manager_(new QtVariantPropertyManager()), factory_(new QtVariantEditorFactory())
  {
    loading_ = true;
    QtVariantProperty *item;
    QtVariantProperty *sub_item;
    double p_norm, r_norm;
    urdf::Pose origin;

    QObject::connect(manager_, SIGNAL(valueChanged(QtProperty *, const QVariant &)),
              this, SLOT(onValueChanged(QtProperty *, const QVariant &)));

    top_item_ = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), tr("Inertial"));

    item = manager_->addProperty(QVariant::Double, tr("Mass (kg)"));
    item->setAttribute(Common::attributeStr(Minimum), 0.0);
    top_item_->addSubProperty(item);

    item = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), tr("Inertia"));
    sub_item = manager_->addProperty(QVariant::Double, tr("Ixx"));
    item->addSubProperty(sub_item);
    sub_item = manager_->addProperty(QVariant::Double, tr("Iyy"));
    item->addSubProperty(sub_item);
    sub_item = manager_->addProperty(QVariant::Double, tr("Izz"));
    item->addSubProperty(sub_item);
    sub_item = manager_->addProperty(QVariant::Double, tr("Ixy"));
    item->addSubProperty(sub_item);
    sub_item = manager_->addProperty(QVariant::Double, tr("Ixz"));
    item->addSubProperty(sub_item);
    sub_item = manager_->addProperty(QVariant::Double, tr("Iyz"));
    item->addSubProperty(sub_item);
    top_item_->addSubProperty(item);

    origin = inertial_->origin;
    p_norm = origin.position.x * origin.position.x;
    p_norm += origin.position.y * origin.position.y;
    p_norm += origin.position.z * origin.position.z;
    r_norm = origin.rotation.x * origin.rotation.x;
    r_norm += origin.rotation.y * origin.rotation.y;
    r_norm += origin.rotation.z * origin.rotation.z;
    if (p_norm > 0.0 || r_norm > 0.0)
    {
      origin_property_.reset(new OriginProperty(inertial_->origin));
      QObject::connect(origin_property_.get(), SIGNAL(valueChanged(QtProperty *, const QVariant &)),
                this, SLOT(onChildValueChanged(QtProperty *, const QVariant &)));

      top_item_->addSubProperty(origin_property_->getTopItem());
    }
    loading_ = false;
  }

  LinkInertialProperty::~LinkInertialProperty()
  {
    delete manager_;
    delete factory_;
  }

  void LinkInertialProperty::loadData()
  {
    loading_ = true;
    QtVariantProperty *item;
    QString name;

    //Get Mass, since its the first one
    item = static_cast<QtVariantProperty *>(top_item_->subProperties()[0]);
    name = item->propertyName();
      if (name == "Mass (kg)")
        item->setValue(inertial_->mass);

    //Remaining sub-properties are for Inertial values
    QList<QtProperty *> sub_items = top_item_->subProperties()[1]->subProperties();
    for (int i = 0; i < sub_items.length(); ++i)
    {
      item = static_cast<QtVariantProperty *>(sub_items[i]);
      name = item->propertyName();
      if (name == "Mass (kg)")
        item->setValue(inertial_->mass);
      else if (name == "Ixx")
        item->setValue(inertial_->ixx);
      else if (name == "Iyy")
        item->setValue(inertial_->iyy);
      else if (name == "Izz")
        item->setValue(inertial_->izz);
      else if (name == "Ixy")
        item->setValue(inertial_->ixy);
      else if (name == "Ixz")
        item->setValue(inertial_->ixz);
      else if (name == "Iyz")
        item->setValue(inertial_->iyz);
    }

    if (origin_property_)
      origin_property_->loadData();

    loading_ = false;
  }

  bool LinkInertialProperty::hasOriginProperty()
  {
    return (origin_property_ != NULL);
  }

  void LinkInertialProperty::createOriginProperty()
  {
    if (!origin_property_)
    {
      origin_property_.reset(new OriginProperty(inertial_->origin));
      top_item_->addSubProperty(origin_property_->getTopItem());
    }
  }

  void LinkInertialProperty::loadFactoryForManager(QtTreePropertyBrowserSharedPtr& property_editor)
  {
    property_editor->setFactoryForManager(manager_, factory_);
    if (origin_property_)
      origin_property_->loadFactoryForManager(property_editor);
  }

  void LinkInertialProperty::onValueChanged(QtProperty *property, const QVariant &val)
  {
    if (loading_)
      return;

    QString name = property->propertyName();
    if (name == "Mass (kg)")
      inertial_->mass = val.toDouble();
    else if (name == "Ixx")
      inertial_->ixx = val.toDouble();
    else if (name == "Iyy")
      inertial_->iyy = val.toDouble();
    else if (name == "Izz")
      inertial_->izz = val.toDouble();
    else if (name == "Ixy")
      inertial_->ixy = val.toDouble();
    else if (name == "Ixz")
      inertial_->ixz = val.toDouble();
    else if (name == "Iyz")
      inertial_->iyz = val.toDouble();

    emit LinkInertialProperty::valueChanged(property, val);
  }

  void LinkInertialProperty::onChildValueChanged(QtProperty *property, const QVariant &val)
  {
    if (loading_)
      return;

    emit LinkInertialProperty::valueChanged(property, val);
  }
}
