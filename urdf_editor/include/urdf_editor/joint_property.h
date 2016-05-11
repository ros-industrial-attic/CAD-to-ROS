#ifndef __JOINT_PROPERTY_H__
#define __JOINT_PROPERTY_H__

#include <QtCore>

#include <urdf_editor/common.h>
#include <urdf_editor/property_types.h>
#include <urdf_editor/qt_types.h>
#include <urdf_editor/urdf_types_ext.h>


namespace urdf_editor
{
  class JointProperty : public QObject
  {
    Q_OBJECT
  public:
    JointProperty(urdf::JointSharedPtr joint, QStringList &link_names, QStringList &joint_names);
    ~JointProperty();

    void loadProperty(boost::shared_ptr<QtTreePropertyBrowser> property_editor);

    void loadData();
    
    /*! Check if has origin property */
    bool hasOriginProperty();
    
    /*! Create origin property */
    void createOriginProperty();
   
    /*! Get the Origin Property */
    OriginPropertySharedPtr getOriginProperty();
    
    /*! Check if has axis property */
    bool hasAxisProperty();
    
    /*! Create axis property */
    void createAxisProperty();
   
    /*! Get the axis Property */
    JointAxisPropertySharedPtr getAxisProperty();
    
    /*! Check if has limits property */
    bool hasLimitsProperty();
    
    /*! Create limits property */
    void createLimitsProperty();
   
    /*! Get the limits Property */
    JointLimitsPropertySharedPtr getLimitsProperty();
    
    /*! Check if has calibration property */
    bool hasCalibrationProperty();
    
    /*! Create calibration property */
    void createCalibrationProperty();
   
    /*! Get the calibration Property */
    JointCalibrationPropertySharedPtr getCalibrationProperty();
    
    /*! Check if has dyanmics property */
    bool hasDynamicsProperty();
    
    /*! Create dynamics property */
    void createDynamicsProperty();
   
    /*! Get the dynamics Property */
    JointDynamicsPropertySharedPtr getDynamicsProperty();
    
    /*! Check if has mimic property */
    bool hasMimicProperty();
    
    /*! Create mimic property */
    void createMimicProperty();
   
    /*! Get the mimic Property */
    JointMimicPropertySharedPtr getMimicProperty();
    
    /*! Check if has safety property */
    bool hasSafetyProperty();
    
    /*! Create safety property */
    void createSafetyProperty();
   
    /*! Get the safety Property */
    JointSafetyPropertySharedPtr getSafetyProperty();
    
    /*! Get the joint name */
    QString getName();

    /*! Get the child link name*/
    QString getChildLinkName();

    /*! Get the parent link name */
    QString getParentLinkName();
    
    /*! Set the child link name*/
    bool setChildLinkName(QString link_name);

    /*! Set the parent link name */
    bool setParentLinkName(QString link_name);
    

  private slots:
    void onValueChanged(QtProperty *property, const QVariant &val);
    void onChildValueChanged(QtProperty *property, const QVariant &val);

  signals:
    void jointNameChanged(JointProperty *property, const QVariant &val);
    void valueChanged();

  private:
    urdf::JointSharedPtr joint_;
    QtVariantPropertyManager *manager_;
    QtVariantEditorFactory *factory_;
    bool loading_;
    QStringList &link_names_;
    QStringList &joint_names_;
    JointLimitsPropertySharedPtr limits_property_;
    JointDynamicsPropertySharedPtr dynamics_property_;
    JointCalibrationPropertySharedPtr calibration_property_;
    JointMimicPropertySharedPtr mimic_property_;
    JointSafetyPropertySharedPtr safety_property_;
    JointAxisPropertySharedPtr axis_property_;
    OriginPropertySharedPtr origin_property_;
    QtVariantProperty *name_item_;
    QtVariantProperty *type_item_;
    QtVariantProperty *parent_item_;
    QtVariantProperty *child_item_;
  };
}

#endif // __JOINT_PROPERTY_H__
