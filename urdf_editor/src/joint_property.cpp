
#include <qttreepropertybrowser.h>
#include <qtvariantproperty.h>

#include <urdf_editor/joint_property.h>

#include <urdf_editor/origin_property.h>
#include <urdf_editor/joint_axis_property.h>
#include <urdf_editor/joint_calibration_property.h>
#include <urdf_editor/joint_dynamics_property.h>
#include <urdf_editor/joint_limits_property.h>
#include <urdf_editor/joint_mimic_property.h>
#include <urdf_editor/joint_safety_property.h>

#include <urdf_model/pose.h>
#include <urdf_model/joint.h>


namespace urdf_editor
{
  JointProperty::JointProperty(urdf::JointSharedPtr joint, QStringList &link_names, QStringList &joint_names): joint_(joint), manager_(new QtVariantPropertyManager()), factory_(new QtVariantEditorFactory()), link_names_(link_names), joint_names_(joint_names)
  {
    loading_ = true;
    double p_norm, r_norm;
    urdf::Pose origin;

    QObject::connect(manager_, SIGNAL(valueChanged(QtProperty *, const QVariant &)),
              this, SLOT(onValueChanged(QtProperty *, const QVariant &)));

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
    {
      origin_property_.reset(new OriginProperty(joint->parent_to_joint_origin_transform));
      QObject::connect(origin_property_.get(), SIGNAL(valueChanged(QtProperty *, const QVariant &)),
                this, SLOT(onChildValueChanged(QtProperty *, const QVariant &)));
    }

    p_norm = joint_->axis.x * joint_->axis.x;
    p_norm += (joint_->axis.y * joint_->axis.y);
    p_norm += (joint_->axis.z * joint_->axis.z);
    if (p_norm > 0.0)
    {
      axis_property_.reset(new JointAxisProperty(joint_->axis));
      QObject::connect(axis_property_.get(), SIGNAL(valueChanged(QtProperty *, const QVariant &)),
                this, SLOT(onChildValueChanged(QtProperty *, const QVariant &)));
    }

    if (joint_->calibration)
    {
      calibration_property_.reset(new JointCalibrationProperty(joint_->calibration));
      QObject::connect(calibration_property_.get(), SIGNAL(valueChanged(QtProperty *, const QVariant &)),
                this, SLOT(onChildValueChanged(QtProperty *, const QVariant &)));
    }

    if (joint_->dynamics)
    {
      dynamics_property_.reset(new JointDynamicsProperty(joint_->dynamics));
      QObject::connect(dynamics_property_.get(), SIGNAL(valueChanged(QtProperty *, const QVariant &)),
                this, SLOT(onChildValueChanged(QtProperty *, const QVariant &)));
    }

    if (joint_->limits)
    {
      limits_property_.reset(new JointLimitsProperty(joint_->limits));
      QObject::connect(limits_property_.get(), SIGNAL(valueChanged(QtProperty *, const QVariant &)),
                this, SLOT(onChildValueChanged(QtProperty *, const QVariant &)));
    }

    if (joint_->mimic)
    {
      mimic_property_.reset(new JointMimicProperty(joint_->mimic, joint_names_));
      QObject::connect(mimic_property_.get(), SIGNAL(valueChanged(QtProperty *, const QVariant &)),
                this, SLOT(onChildValueChanged(QtProperty *, const QVariant &)));
    }

    if (joint_->safety)
    {
      safety_property_.reset(new JointSafetyProperty(joint_->safety));
      QObject::connect(safety_property_.get(), SIGNAL(valueChanged(QtProperty *, const QVariant &)),
                this, SLOT(onChildValueChanged(QtProperty *, const QVariant &)));
    }

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

  void JointProperty::onValueChanged(QtProperty *property, const QVariant &val)
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

    emit JointProperty::valueChanged();
  }

  void JointProperty::onChildValueChanged(QtProperty *property, const QVariant &val)
  {
    if (loading_)
      return;

    emit JointProperty::valueChanged();
  }

   /*!
   *@brief Checks if the Joint has an origin property defined
   * 
   *@return returns true if origin property defined
   */
  bool JointProperty::hasOriginProperty()
  {
    return (origin_property_ != NULL);
  }

  /*!
   *@brief Creates the origin property
   * 
   */
  void JointProperty::createOriginProperty()
  {
    if(! origin_property_)
    {
      origin_property_.reset(new OriginProperty(joint_->parent_to_joint_origin_transform));
    }
  }
  
   /*!
   *@brief Get the origin property Object
   * 
   *@return OriginPropertySharedPtr
   */
  OriginPropertySharedPtr JointProperty::getOriginProperty()
  {
    return origin_property_;
  }
  
   /*!
   *@brief Checks if the Joint has an origin property defined
   * 
   *@return returns true if axis property defined
   */
  bool JointProperty::hasAxisProperty()
  {
    return (axis_property_ != NULL);
  }

  /*!
   *@brief Creates the axis property
   * 
   */
  void JointProperty::createAxisProperty()
  {
    if(! axis_property_)
    {
      axis_property_.reset(new JointAxisProperty(joint_->axis));
    }
  }
  
   /*!
   *@brief Get the axis property Object
   * 
   *@return JointAxisPropertySharedPtr
   */
  JointAxisPropertySharedPtr JointProperty::getAxisProperty()
  {
    return axis_property_;
  }
  
   /*!
   *@brief Checks if the Joint has a limits property defined
   * 
   *@return returns true if limits property defined
   */
  bool JointProperty::hasLimitsProperty()
  {
    return (limits_property_ != NULL);
  }

  /*!
   *@brief Creates the limits property
   * 
   */
  void JointProperty::createLimitsProperty()
  {
    if(!joint_->limits)
    {
      joint_->limits.reset(new urdf::JointLimits());
      limits_property_.reset(new JointLimitsProperty(joint_->limits));
    }
  }
  
   /*!
   *@brief Get the limts property Object
   * 
   *@return JointLimitsPropertySharedPtr
   */
  JointLimitsPropertySharedPtr JointProperty::getLimitsProperty()
  {
    return limits_property_;
  }
  
  /*!
   *@brief Checks if the Joint has a calibration property defined
   * 
   *@return returns true if calibration property defined
   */
  bool JointProperty::hasCalibrationProperty()
  {
    return (calibration_property_ != NULL);
  }

  /*!
   *@brief Creates the calibration property
   * 
   */
  void JointProperty::createCalibrationProperty()
  {
    if(!joint_->calibration)
    {
      joint_->calibration.reset(new urdf::JointCalibration());
      calibration_property_.reset(new JointCalibrationProperty(joint_->calibration));
    }
  }
  
   /*!
   *@brief Get the calibration property Object
   * 
   *@return JointCalibrationPropertySharedPtr
   */
  JointCalibrationPropertySharedPtr JointProperty::getCalibrationProperty()
  {
    return calibration_property_;
  }
  
  /*!
   *@brief Checks if the Joint has a dynamics property defined
   * 
   *@return returns true if dynamics property defined
   */
  bool JointProperty::hasDynamicsProperty()
  {
    return (dynamics_property_ != NULL);
  }

  /*!
   *@brief Creates the dynamics property
   * 
   */
  void JointProperty::createDynamicsProperty()
  {
    if(!joint_->dynamics)
    {
      joint_->dynamics.reset(new urdf::JointDynamics());
      dynamics_property_.reset(new JointDynamicsProperty(joint_->dynamics));
    }
  }
  
   /*!
   *@brief Get the dynamics property Object
   * 
   *@return JointDynamicsPropertySharedPtr
   */
  JointDynamicsPropertySharedPtr JointProperty::getDynamicsProperty()
  {
    return dynamics_property_;
  }
  
  /*!
   *@brief Checks if the Joint has a mimic property defined
   * 
   *@return returns true if mimic property defined
   */
  bool JointProperty::hasMimicProperty()
  {
    return (mimic_property_ != NULL);
  }

  /*!
   *@brief Creates the mimic property
   * 
   */
  void JointProperty::createMimicProperty()
  {
    if(!joint_->mimic)
    {
      joint_->mimic.reset(new urdf::JointMimic());
      mimic_property_.reset(new JointMimicProperty(joint_->mimic, joint_names_));
    }
  }
  
   /*!
   *@brief Get the mimic property Object
   * 
   *@return JointMimicPropertySharedPtr
   */
  JointMimicPropertySharedPtr JointProperty::getMimicProperty()
  {
    return mimic_property_;
  }
  
   /*!
   *@brief Checks if the Joint has a safety property defined
   * 
   *@return returns true if safety property defined
   */
  bool JointProperty::hasSafetyProperty()
  { 
    return (safety_property_ != NULL);
  }

  /*!
   *@brief Creates the safety property
   * 
   */
  void JointProperty::createSafetyProperty()
  {
    if(!joint_->safety)
    {
      joint_->safety.reset(new urdf::JointSafety());
      safety_property_.reset(new JointSafetyProperty(joint_->safety));
    }
  }
  
   /*!
   *@brief Get the safety property Object
   * 
   *@return JointSafetyPropertySharedPtr
   */
  JointSafetyPropertySharedPtr JointProperty::getSafetyProperty()
  {
    return safety_property_;
  }
}
