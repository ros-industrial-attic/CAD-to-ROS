#include "include/urdf_editor/joint_property.h"

namespace urdf_editor
{
  //Origin Property
  OriginProperty::OriginProperty(urdf::Pose &origin): origin_(origin), manager_(new QtVariantPropertyManager()), factory_(new QtVariantEditorFactory())
  {
    loading_ = true;
    QtVariantProperty *item;
    QtVariantProperty *sub_item;

    QObject::connect(manager_, SIGNAL(valueChanged(QtProperty *, const QVariant &)),
              this, SLOT(originValueChanged(QtProperty *, const QVariant &)));

    top_item_ = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), QTranslator::tr("Origin"));

    // Create position properties
    item = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), QTranslator::tr("Position (m)"));
    sub_item = manager_->addProperty(QVariant::Double, QTranslator::tr("X"));
    sub_item->setAttribute(Common::attributeStr(Decimals), 6);
    item->addSubProperty(sub_item);
    sub_item = manager_->addProperty(QVariant::Double, QTranslator::tr("Y"));
    sub_item->setAttribute(Common::attributeStr(Decimals), 6);
    item->addSubProperty(sub_item);
    sub_item = manager_->addProperty(QVariant::Double, QTranslator::tr("Z"));
    sub_item->setAttribute(Common::attributeStr(Decimals), 6);
    item->addSubProperty(sub_item);
    top_item_->addSubProperty(item);

    // Create orientation properties
    item = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), QTranslator::tr("Orientation (rad)"));
    sub_item = manager_->addProperty(QVariant::Double, QTranslator::tr("Roll"));
    sub_item->setAttribute(Common::attributeStr(Decimals), 6);
    item->addSubProperty(sub_item);
    sub_item = manager_->addProperty(QVariant::Double, QTranslator::tr("Pitch"));
    sub_item->setAttribute(Common::attributeStr(Decimals), 6);
    item->addSubProperty(sub_item);
    sub_item = manager_->addProperty(QVariant::Double, QTranslator::tr("Yaw"));
    sub_item->setAttribute(Common::attributeStr(Decimals), 6);
    item->addSubProperty(sub_item);
    top_item_->addSubProperty(item);

    loading_ = false;
  }

  OriginProperty::~OriginProperty()
  {
    delete manager_;
    delete factory_;
  }

  void OriginProperty::loadData()
  {
    loading_ = true;
    QtVariantProperty *item;
    QString name;
    double r, p, y;
    QList<QtProperty *> sub_items;

    sub_items = top_item_->subProperties()[0]->subProperties();
    for (int i = 0; i < sub_items.length(); ++i)
    {
      item = static_cast<QtVariantProperty *>(sub_items[i]);
      name = item->propertyName();
      if (name == "X")
        item->setValue(origin_.position.x);
      else if (name == "Y")
        item->setValue(origin_.position.y);
      else if (name == "Z")
        item->setValue(origin_.position.z);
    }

    sub_items = top_item_->subProperties()[1]->subProperties();
    for (int i = 0; i < sub_items.length(); ++i)
    {
      item = static_cast<QtVariantProperty *>(sub_items[i]);
      name = item->propertyName();

      origin_.rotation.getRPY(r, p, y);

      if (name == "Roll")
        item->setValue(r);
      else if (name == "Pitch")
        item->setValue(p);
      else if (name == "Yaw")
        item->setValue(y);
    }

    loading_ = false;
  }

  void OriginProperty::loadFactoryForManager(boost::shared_ptr<QtTreePropertyBrowser> property_editor)
  {
    property_editor->setFactoryForManager(manager_, factory_);
  }

  void OriginProperty::originValueChanged(QtProperty *property, const QVariant &val)
  {
    if (loading_)
      return;

    double r, p, y;
    QString name = property->propertyName();

    if (name == "X")
      origin_.position.x = val.toDouble();
    else if (name == "Y")
      origin_.position.y = val.toDouble();
    else if (name == "Z")
      origin_.position.z = val.toDouble();
    else if (name == "Roll" || name == "Pitch" || name == "Yaw")
    {
      origin_.rotation.getRPY(r, p, y);

      if (name == "Roll")
        r = val.toDouble();
      else if (name == "Pitch")
        p = val.toDouble();
      else if (name == "Yaw")
        y = val.toDouble();

      origin_.rotation.setFromRPY(r, p, y);
    }
  }

  //Joint Axis Property
  JointAxisProperty::JointAxisProperty(urdf::Vector3 &axis): axis_(axis), manager_(new QtVariantPropertyManager()), factory_(new QtVariantEditorFactory())
  {
    loading_ = true;
    QtVariantProperty *item;

    QObject::connect(manager_, SIGNAL(valueChanged(QtProperty *, const QVariant &)),
              this, SLOT(jointAxisValueChanged(QtProperty *, const QVariant &)));

    top_item_ = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), tr("Axis"));
    item = manager_->addProperty(QVariant::Double, tr("X"));
    item->setAttribute(Common::attributeStr(Decimals), 6);
    top_item_->addSubProperty(item);
    item = manager_->addProperty(QVariant::Double, tr("Y"));
    item->setAttribute(Common::attributeStr(Decimals), 6);
    top_item_->addSubProperty(item);
    item = manager_->addProperty(QVariant::Double, tr("Z"));
    item->setAttribute(Common::attributeStr(Decimals), 6);
    top_item_->addSubProperty(item);

    loading_ = false;
  }

  JointAxisProperty::~JointAxisProperty()
  {
    delete manager_;
    delete factory_;
  }

  void JointAxisProperty::loadData()
  {
    loading_ = true;
    QtVariantProperty *item;
    QString name;
    QList<QtProperty *> sub_items = top_item_->subProperties();
    for (int i = 0; i < sub_items.length(); ++i)
    {
      item = static_cast<QtVariantProperty *>(sub_items[i]);
      name = item->propertyName();
      if (name == "X")
        item->setValue(axis_.x);
      else if (name == "Y")
        item->setValue(axis_.y);
      else if (name == "Z")
        item->setValue(axis_.z);
    }
    loading_ = false;
  }

  void JointAxisProperty::loadFactoryForManager(boost::shared_ptr<QtTreePropertyBrowser> property_editor)
  {
    property_editor->setFactoryForManager(manager_, factory_);
  }

  void JointAxisProperty::jointAxisValueChanged(QtProperty *property, const QVariant &val)
  {
    if (loading_)
      return;

    QString name = property->propertyName();
    if (name == "X")
      axis_.x = val.toDouble();
    else if (name == "Y")
      axis_.y = val.toDouble();
    else if (name == "Z")
      axis_.z = val.toDouble();
  }

  //Joint Safety Property
  JointSafetyProperty::JointSafetyProperty(boost::shared_ptr<urdf::JointSafety> safety): safety_(safety), manager_(new QtVariantPropertyManager()), factory_(new QtVariantEditorFactory())
  {
    loading_ = true;
    QtVariantProperty *item;

    QObject::connect(manager_, SIGNAL(valueChanged(QtProperty *, const QVariant &)),
              this, SLOT(jointSafetyValueChanged(QtProperty *, const QVariant &)));

    top_item_ = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), tr("Safety Controller"));
    item = manager_->addProperty(QVariant::Double, tr("Soft Lower Limit"));
    item->setAttribute(Common::attributeStr(Decimals), 6);
    top_item_->addSubProperty(item);
    item = manager_->addProperty(QVariant::Double, tr("Soft Upper Limit"));
    item->setAttribute(Common::attributeStr(Decimals), 6);
    top_item_->addSubProperty(item);
    item = manager_->addProperty(QVariant::Double, tr("K Position"));
    item->setAttribute(Common::attributeStr(Decimals), 6);
    top_item_->addSubProperty(item);
    item = manager_->addProperty(QVariant::Double, tr("K Velocity"));
    item->setAttribute(Common::attributeStr(Decimals), 6);
    top_item_->addSubProperty(item);

    loading_ = false;
  }

  JointSafetyProperty::~JointSafetyProperty()
  {
    delete manager_;
    delete factory_;
  }

  void JointSafetyProperty::loadData()
  {
    loading_ = true;
    QtVariantProperty *item;
    QString name;
    QList<QtProperty *> sub_items = top_item_->subProperties();
    for (int i = 0; i < sub_items.length(); ++i)
    {
      item = static_cast<QtVariantProperty *>(sub_items[i]);
      name = item->propertyName();
      if (name == "Soft Lower Limit")
        item->setValue(safety_->soft_lower_limit);
      else if (name == "Soft Upper Limit")
        item->setValue(safety_->soft_upper_limit);
      else if (name == "K Position")
        item->setValue(safety_->k_position);
      else if (name == "K Velocity")
        item->setValue(safety_->k_velocity);
    }
    loading_ = false;
  }

  void JointSafetyProperty::loadFactoryForManager(boost::shared_ptr<QtTreePropertyBrowser> property_editor)
  {
    property_editor->setFactoryForManager(manager_, factory_);
  }

  void JointSafetyProperty::jointSafetyValueChanged(QtProperty *property, const QVariant &val)
  {
    if (loading_)
      return;

    QString name = property->propertyName();
    if (name == "Soft Lower Limit")
      safety_->soft_lower_limit = val.toDouble();
    else if (name == "Soft Upper Limit")
      safety_->soft_upper_limit = val.toDouble();
    else if (name == "K Position")
      safety_->k_position = val.toDouble();
    else if (name == "K Velocity")
      safety_->k_velocity = val.toDouble();
  }

  //Joint Mimic Property
  JointMimicProperty::JointMimicProperty(boost::shared_ptr<urdf::JointMimic> mimic, QStringList &joint_names): mimic_(mimic), manager_(new QtVariantPropertyManager()), factory_(new QtVariantEditorFactory()), joint_names_(joint_names)
  {
    loading_ = true;
    QtVariantProperty *item;

    QObject::connect(manager_, SIGNAL(valueChanged(QtProperty *, const QVariant &)),
              this, SLOT(jointMimicValueChanged(QtProperty *, const QVariant &)));

    top_item_ = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), tr("Mimic"));
    item = manager_->addProperty(QtVariantPropertyManager::enumTypeId(), tr("Joint"));
    item->setAttribute(Common::attributeStr(EnumNames), joint_names_);
    top_item_->addSubProperty(item);
    item = manager_->addProperty(QVariant::Double, tr("Multiplier"));
    item->setAttribute(Common::attributeStr(Decimals), 6);
    top_item_->addSubProperty(item);
    item = manager_->addProperty(QVariant::Double, tr("Offset"));
    item->setAttribute(Common::attributeStr(Decimals), 6);
    top_item_->addSubProperty(item);

    loading_ = false;
  }

  JointMimicProperty::~JointMimicProperty()
  {
    delete manager_;
    delete factory_;
  }

  void JointMimicProperty::loadData()
  {
    loading_ = true;
    QtVariantProperty *item;
    QString name;
    QList<QtProperty *> sub_items = top_item_->subProperties();
    for (int i = 0; i < sub_items.length(); ++i)
    {
      item = static_cast<QtVariantProperty *>(sub_items[i]);
      name = item->propertyName();
      if (name == "Joint")
      {
        item->setAttribute(Common::attributeStr(EnumNames), joint_names_);
        item->setValue(joint_names_.indexOf(QString::fromStdString(mimic_->joint_name)));
      }
      else if (name == "Multiplier")
        item->setValue(mimic_->multiplier);
      else if (name == "Offset")
        item->setValue(mimic_->offset);
    }
    loading_ = false;
  }

  void JointMimicProperty::loadFactoryForManager(boost::shared_ptr<QtTreePropertyBrowser> property_editor)
  {
    property_editor->setFactoryForManager(manager_, factory_);
  }

  void JointMimicProperty::jointMimicValueChanged(QtProperty *property, const QVariant &val)
  {
    if (loading_)
      return;

    QString name = property->propertyName();
    if (name == "Joint")
      mimic_->joint_name = val.toString().toStdString();
    else if (name == "Multiplier")
      mimic_->multiplier = val.toDouble();
    else if (name == "Offset")
      mimic_->offset = val.toDouble();
  }

  //Joint Calibration Property
  JointCalibrationProperty::JointCalibrationProperty(boost::shared_ptr<urdf::JointCalibration> calibration): calibration_(calibration), manager_(new QtVariantPropertyManager()), factory_(new QtVariantEditorFactory())
  {
    loading_ = true;
    QtVariantProperty *item;

    QObject::connect(manager_, SIGNAL(valueChanged(QtProperty *, const QVariant &)),
              this, SLOT(jointCalibrationValueChanged(QtProperty *, const QVariant &)));

    top_item_ = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), tr("Calibration"));
    item = manager_->addProperty(QVariant::Double, tr("Rising"));
    item->setAttribute(Common::attributeStr(Minimum), 0.0);
    item->setAttribute(Common::attributeStr(Decimals), 6);
    top_item_->addSubProperty(item);
    item = manager_->addProperty(QVariant::Double, tr("Falling"));
    item->setAttribute(Common::attributeStr(Minimum), 0.0);
    item->setAttribute(Common::attributeStr(Decimals), 6);
    top_item_->addSubProperty(item);

    loading_ = false;
  }

  JointCalibrationProperty::~JointCalibrationProperty()
  {
    delete manager_;
    delete factory_;
  }

  void JointCalibrationProperty::loadData()
  {
    loading_ = true;
    QtVariantProperty *item;
    QString name;
    QList<QtProperty *> sub_items = top_item_->subProperties();
    for (int i = 0; i < sub_items.length(); ++i)
    {
      item = static_cast<QtVariantProperty *>(sub_items[i]);
      name = item->propertyName();
      if (name == "Rising")
        item->setValue(*calibration_->rising);
      else if (name == "Falling")
        item->setValue(*calibration_->falling);
    }
    loading_ = false;
  }

  void JointCalibrationProperty::loadFactoryForManager(boost::shared_ptr<QtTreePropertyBrowser> property_editor)
  {
    property_editor->setFactoryForManager(manager_, factory_);
  }

  void JointCalibrationProperty::jointCalibrationValueChanged(QtProperty *property, const QVariant &val)
  {
    if (loading_)
      return;

    QString name = property->propertyName();
    if (name == "Rising")
      *calibration_->rising = val.toDouble();
    else if (name == "Falling")
      *calibration_->falling = val.toDouble();
  }

  //Joint Dynamics Property
  JointDynamicsProperty::JointDynamicsProperty(boost::shared_ptr<urdf::JointDynamics> dynamics): dynamics_(dynamics), manager_(new QtVariantPropertyManager()), factory_(new QtVariantEditorFactory())
  {
    loading_ = true;
    QtVariantProperty *item;

    QObject::connect(manager_, SIGNAL(valueChanged(QtProperty *, const QVariant &)),
              this, SLOT(jointDynamicsValueChanged(QtProperty *, const QVariant &)));

    top_item_ = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), tr("Dynamics"));
    item = manager_->addProperty(QVariant::Double, tr("Damping"));
    item->setAttribute(Common::attributeStr(Minimum), 0.0);
    item->setAttribute(Common::attributeStr(Decimals), 6);
    top_item_->addSubProperty(item);
    item = manager_->addProperty(QVariant::Double, tr("Friction"));
    item->setAttribute(Common::attributeStr(Minimum), 0.0);
    item->setAttribute(Common::attributeStr(Decimals), 6);
    top_item_->addSubProperty(item);

    loading_ = false;
  }

  JointDynamicsProperty::~JointDynamicsProperty()
  {
    delete manager_;
    delete factory_;
  }

  void JointDynamicsProperty::loadData()
  {
    loading_ = true;
    QtVariantProperty *item;
    QString name;
    QList<QtProperty *> sub_items = top_item_->subProperties();
    for (int i = 0; i < sub_items.length(); ++i)
    {
      item = static_cast<QtVariantProperty *>(sub_items[i]);
      name = item->propertyName();
      if (name == "Damping")
        item->setValue(dynamics_->damping);
      else if (name == "Friction")
        item->setValue(dynamics_->friction);
    }
    loading_ = false;
  }

  void JointDynamicsProperty::loadFactoryForManager(boost::shared_ptr<QtTreePropertyBrowser> property_editor)
  {
    property_editor->setFactoryForManager(manager_, factory_);
  }

  void JointDynamicsProperty::jointDynamicsValueChanged(QtProperty *property, const QVariant &val)
  {
    if (loading_)
      return;

    QString name = property->propertyName();
    if (name == "Damping")
      dynamics_->damping = val.toDouble();
    else if (name == "Friction")
      dynamics_->friction = val.toDouble();
  }

  //Joint Limits Property
  JointLimitsProperty::JointLimitsProperty(boost::shared_ptr<urdf::JointLimits> limits): limits_(limits), manager_(new QtVariantPropertyManager()), factory_(new QtVariantEditorFactory())
  {
    loading_ = true;
    QtVariantProperty *item;

    QObject::connect(manager_, SIGNAL(valueChanged(QtProperty *, const QVariant &)),
              this, SLOT(jointLimitsValueChanged(QtProperty *, const QVariant &)));

    top_item_ = manager_->addProperty(QtVariantPropertyManager::groupTypeId(), tr("Limit"));
    item = manager_->addProperty(QVariant::Double, tr("Lower"));
    item->setValue(limits_->lower);
    item->setAttribute(Common::attributeStr(Decimals), 6);
    top_item_->addSubProperty(item);
    item = manager_->addProperty(QVariant::Double, tr("Upper"));
    item->setValue(limits_->upper);
    item->setAttribute(Common::attributeStr(Decimals), 6);
    top_item_->addSubProperty(item);
    item = manager_->addProperty(QVariant::Double, tr("Effort"));
    item->setValue(limits_->effort);
    item->setAttribute(Common::attributeStr(Minimum), 0.0);
    item->setAttribute(Common::attributeStr(Decimals), 6);
    item = manager_->addProperty(QVariant::Double, tr("Velocity"));
    item->setValue(limits_->velocity);
    item->setAttribute(Common::attributeStr(Minimum), 0.0);
    item->setAttribute(Common::attributeStr(Decimals), 6);
    top_item_->addSubProperty(item);

    loading_ = false;
  }

  JointLimitsProperty::~JointLimitsProperty()
  {
    delete manager_;
    delete factory_;
  }

  void JointLimitsProperty::loadData()
  {
    loading_ = true;
    QtVariantProperty *item;
    QString name;
    QList<QtProperty *> sub_items = top_item_->subProperties();
    for (int i = 0; i < sub_items.length(); ++i)
    {
      item = static_cast<QtVariantProperty *>(sub_items[i]);
      name = item->propertyName();
      if (name == "Lower")
        item->setValue(limits_->lower);
      else if (name == "Upper")
        item->setValue(limits_->upper);
      else if (name == "Effort")
        item->setValue(limits_->effort);
      else if (name == "Velocity")
        item->setValue(limits_->velocity);
    }
    loading_ = false;
  }

  void JointLimitsProperty::loadFactoryForManager(boost::shared_ptr<QtTreePropertyBrowser> property_editor)
  {
    property_editor->setFactoryForManager(manager_, factory_);
  }

  void JointLimitsProperty::jointLimitsValueChanged(QtProperty *property, const QVariant &val)
  {
    if (loading_)
      return;

    QString name = property->propertyName();
    if (name == "Lower")
      limits_->lower = val.toDouble();
    else if (name == "Upper")
      limits_->upper = val.toDouble();
    else if (name == "Effort")
      limits_->effort = val.toDouble();
    else if (name == "Velocity")
      limits_->velocity = val.toDouble();
  }

  // Joint Property
  JointProperty::JointProperty(boost::shared_ptr<urdf::Joint> joint, QStringList &link_names, QStringList &joint_names): joint_(joint), manager_(new QtVariantPropertyManager()), factory_(new QtVariantEditorFactory()), link_names_(link_names), joint_names_(joint_names)
  {
    loading_ = true;
    double p_norm, r_norm;
    urdf::Pose origin;

    QObject::connect(manager_, SIGNAL(valueChanged(QtProperty *, const QVariant &)),
              this, SLOT(jointValueChanged(QtProperty *, const QVariant &)));

    name_item_ = manager_->addProperty(QVariant::String, tr("Name"));

    //UNKNOWN, REVOLUTE, CONTINUOUS, PRISMATIC, FLOATING, PLANAR, FIXED
    type_item_ = manager_->addProperty(QtVariantPropertyManager::enumTypeId(), tr("Type"));
    type_item_->setAttribute(Common::attributeStr(EnumNames), QStringList() << tr("Unknown") << tr("Revolute") << tr("Continuous") << tr("Prismatic") <<  tr("Floating") << tr("Planar") << tr("Fixed"));

    parent_item_ = manager_->addProperty(QVariant::String, tr("Parent"));
    parent_item_->setAttribute(Common::attributeStr(ReadOnly), true);

    child_item_ = manager_->addProperty(QtVariantPropertyManager::enumTypeId(), tr("Child"));
    child_item_->setAttribute(Common::attributeStr(EnumNames), link_names_);

    origin = joint_->parent_to_joint_origin_transform;
    p_norm = origin.position.x * origin.position.x;
    p_norm += origin.position.y * origin.position.y;
    p_norm += origin.position.z * origin.position.z;
    r_norm = origin.rotation.x * origin.rotation.x;
    r_norm += origin.rotation.y * origin.rotation.y;
    r_norm += origin.rotation.z * origin.rotation.z;
    if (p_norm > 0.0 || r_norm > 0.0)
      origin_property_.reset(new OriginProperty(joint->parent_to_joint_origin_transform));

    p_norm = joint_->axis.x * joint_->axis.x;
    p_norm += (joint_->axis.y * joint_->axis.y);
    p_norm += (joint_->axis.z * joint_->axis.z);
    if (p_norm > 0.0)
      axis_property_.reset(new JointAxisProperty(joint_->axis));

    if (joint_->calibration)
      calibration_property_.reset(new JointCalibrationProperty(joint_->calibration));

    if (joint_->dynamics)
      dynamics_property_.reset(new JointDynamicsProperty(joint_->dynamics));

    if (joint_->limits)
      limits_property_.reset(new JointLimitsProperty(joint_->limits));

    if (joint_->mimic)
      mimic_property_.reset(new JointMimicProperty(joint_->mimic, joint_names_));

    if (joint_->safety)
      safety_property_.reset(new JointSafetyProperty(joint_->safety));

    loading_ = false;
  }

  JointProperty::~JointProperty()
  {
    delete manager_;
    delete factory_;
  }

  void JointProperty::loadData()
  {
    loading_ = true;
    name_item_->setValue(QString::fromStdString(joint_->name));
    type_item_->setValue(joint_->type);
    parent_item_->setValue(QString::fromStdString(joint_->parent_link_name));
    child_item_->setAttribute(Common::attributeStr(EnumNames), link_names_);
    child_item_->setValue(link_names_.indexOf(QString::fromStdString(joint_->child_link_name)));

    if (origin_property_)
      origin_property_->loadData();

    if (axis_property_)
      axis_property_->loadData();

    if (limits_property_)
      limits_property_->loadData();

    if (calibration_property_)
      calibration_property_->loadData();

    if (dynamics_property_)
      dynamics_property_->loadData();

    if (mimic_property_)
      mimic_property_->loadData();

    if (safety_property_)
      safety_property_->loadData();

    loading_ = false;
  }

  void JointProperty::loadProperty(boost::shared_ptr<QtTreePropertyBrowser> property_editor)
  {
    loadData();
    property_editor->clear();
    property_editor->setFactoryForManager(manager_, factory_);
    property_editor->addProperty(name_item_);
    property_editor->addProperty(type_item_);
    property_editor->addProperty(parent_item_);
    property_editor->addProperty(child_item_);

    if (origin_property_)
    {
      origin_property_->loadFactoryForManager(property_editor);
      property_editor->addProperty(origin_property_->getTopItem());
    }

    if (axis_property_)
    {
      axis_property_->loadFactoryForManager(property_editor);
      property_editor->addProperty(axis_property_->getTopItem());
    }

    if (limits_property_)
    {
      limits_property_->loadFactoryForManager(property_editor);
      property_editor->addProperty(limits_property_->getTopItem());
    }

    if (calibration_property_)
    {
      calibration_property_->loadFactoryForManager(property_editor);
      property_editor->addProperty(calibration_property_->getTopItem());
    }

    if (dynamics_property_)
    {
      dynamics_property_->loadFactoryForManager(property_editor);
      property_editor->addProperty(dynamics_property_->getTopItem());
    }

    if (mimic_property_)
    {
      mimic_property_->loadFactoryForManager(property_editor);
      property_editor->addProperty(mimic_property_->getTopItem());
    }

    if (safety_property_)
    {
      safety_property_->loadFactoryForManager(property_editor);
      property_editor->addProperty(safety_property_->getTopItem());
    }

  }

  void JointProperty::jointValueChanged(QtProperty *property, const QVariant &val)
  {
    if (loading_)
      return;

    QString name = property->propertyName();
    if (name == "Name")
    {
      joint_->name = val.toString().toStdString();
      emit JointProperty::jointNameChanged(this, val);
    }
    else if (name == "Type")
    {
      //UNKNOWN, REVOLUTE, CONTINUOUS, PRISMATIC, FLOATING, PLANAR, FIXED
      switch (val.toInt())
      {
      case 0:
        joint_->type = urdf::Joint::UNKNOWN;
        break;
      case 1:
        joint_->type = urdf::Joint::REVOLUTE;
        break;
      case 2:
        joint_->type = urdf::Joint::CONTINUOUS;
        break;
      case 3:
        joint_->type = urdf::Joint::PRISMATIC;
        break;
      case 4:
        joint_->type = urdf::Joint::FLOATING;
        break;
      case 5:
        joint_->type = urdf::Joint::PLANAR;
        break;
      case 6:
        joint_->type = urdf::Joint::FIXED;
        break;
      }
    }
    else if (name == "Parent")
    {
      joint_->parent_link_name = val.toString().toStdString();
    }
    else if (name == "Child")
    {
      joint_->child_link_name = link_names_[val.toInt()].toStdString();
      // TODO: When child is changed need to change additional data within joint_
    }
  }


}
