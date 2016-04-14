#ifndef __JOINT_PROPERTY_H__
#define __JOINT_PROPERTY_H__

#include <QtCore>
#include <boost/shared_container_iterator.hpp>

#include <qttreepropertybrowser.h>
#include <qtvariantproperty.h>

#include <urdf_editor/common.h>
#include <urdf_model/joint.h>

#include <urdf_editor/origin_property.h>
#include <urdf_editor/joint_axis_property.h>
#include <urdf_editor/joint_calibration_property.h>
#include <urdf_editor/joint_dynamics_property.h>
#include <urdf_editor/joint_limits_property.h>
#include <urdf_editor/joint_mimic_property.h>
#include <urdf_editor/joint_safety_property.h>


namespace urdf_editor
{
  /*! OriginProperty pointer */
  typedef boost::shared_ptr<OriginProperty> OriginPropertyPtr;
  
  /*! JointAxisProperty pointer */
  typedef boost::shared_ptr<JointAxisProperty> JointAxisPropertyPtr;
  
  /*! JointLimitsProperty pointer */
  typedef boost::shared_ptr<JointLimitsProperty> JointLimitsPropertyPtr;
  
  /*! JointCalibrationProperty pointer */
  typedef boost::shared_ptr<JointCalibrationProperty> JointCalibrationPropertyPtr;
  
  /*! JointDynamicsProperty pointer */
  typedef boost::shared_ptr<JointDynamicsProperty> JointDynamicsPropertyPtr;
  
  /*! JointMimicProperty pointer */
  typedef boost::shared_ptr<JointMimicProperty> JointMimicPropertyPtr;
  
  /*! JointSafetyProperty pointer */
  typedef boost::shared_ptr<JointSafetyProperty> JointSafetyPropertyPtr;
  
  
  class JointProperty : public QObject
  {
    Q_OBJECT
  public:
    JointProperty(boost::shared_ptr<urdf::Joint> joint, QStringList &link_names, QStringList &joint_names);
    ~JointProperty();

    void loadProperty(boost::shared_ptr<QtTreePropertyBrowser> property_editor);

    void loadData();
    
    /*! Check if has origin property */
    bool hasOriginProperty();
    
    /*! Create origin property */
    void createOriginProperty();
   
    /*! Get the Origin Property */
    OriginPropertyPtr getOriginProperty();
    
    /*! Check if has axis property */
    bool hasAxisProperty();
    
    /*! Create axis property */
    void createAxisProperty();
   
    /*! Get the axis Property */
    JointAxisPropertyPtr getAxisProperty();
    
    /*! Check if has limits property */
    bool hasLimitsProperty();
    
    /*! Create limits property */
    void createLimitsProperty();
   
    /*! Get the limits Property */
    JointLimitsPropertyPtr getLimitsProperty();
    
    /*! Check if has calibration property */
    bool hasCalibrationProperty();
    
    /*! Create calibration property */
    void createCalibrationProperty();
   
    /*! Get the calibration Property */
    JointCalibrationPropertyPtr getCalibrationProperty();
    
    /*! Check if has dyanmics property */
    bool hasDynamicsProperty();
    
    /*! Create dynamics property */
    void createDynamicsProperty();
   
    /*! Get the dynamics Property */
    JointDynamicsPropertyPtr getDynamicsProperty();
    
    /*! Check if has mimic property */
    bool hasMimicProperty();
    
    /*! Create mimic property */
    void createMimicProperty();
   
    /*! Get the mimic Property */
    JointMimicPropertyPtr getMimicProperty();
    
    /*! Check if has safety property */
    bool hasSafetyProperty();
    
    /*! Create safety property */
    void createSafetyProperty();
   
    /*! Get the safety Property */
    JointSafetyPropertyPtr getSafetyProperty();
    
    
    

  private slots:
    void onValueChanged(QtProperty *property, const QVariant &val);
    void onChildValueChanged(QtProperty *property, const QVariant &val);

  signals:
    void jointNameChanged(JointProperty *property, const QVariant &val);
    void valueChanged();

  private:
    boost::shared_ptr<urdf::Joint> joint_;
    QtVariantPropertyManager *manager_;
    QtVariantEditorFactory *factory_;
    bool loading_;
    QStringList &link_names_;
    QStringList &joint_names_;
    boost::shared_ptr<JointLimitsProperty> limits_property_;
    boost::shared_ptr<JointDynamicsProperty> dynamics_property_;
    boost::shared_ptr<JointCalibrationProperty> calibration_property_;
    boost::shared_ptr<JointMimicProperty> mimic_property_;
    boost::shared_ptr<JointSafetyProperty> safety_property_;
    boost::shared_ptr<JointAxisProperty> axis_property_;
    boost::shared_ptr<OriginProperty> origin_property_;
    QtVariantProperty *name_item_;
    QtVariantProperty *type_item_;
    QtVariantProperty *parent_item_;
    QtVariantProperty *child_item_;
  };

  typedef boost::shared_ptr<JointProperty> JointPropertyPtr;
}

#endif // __JOINT_PROPERTY_H__
