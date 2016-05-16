#ifndef __JOINT_PROPERTY_H__
#define __JOINT_PROPERTY_H__

#include <QtCore>

#include <urdf_editor/common.h>
#include <urdf_editor/property_types.h>
#include <urdf_editor/qt_types.h>
#include <urdf_editor/urdf_types_ext.h>


namespace urdf_editor
{
  class URDFTransformer;

  class JointProperty : public QObject
  {
    Q_OBJECT
  public:
    JointProperty(urdf::JointSharedPtr joint, QStringList &link_names, QStringList &joint_names);
    ~JointProperty();

    void loadProperty(boost::shared_ptr<QtTreePropertyBrowser> property_editor);

    void loadData();

    /*! Get the joint name */
    QString getName();

    /*! Get the name of the parent link */
    QString getParent() const;

    /*! Get the name of the child link */
    QString getChild() const;

    /*! Set the parent link name */
    bool setParent(const QString &link_name);
    
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

  private slots:
    void onValueChanged(QtProperty *property, const QVariant &val);
    void onOriginChanged(QtProperty *property, const QVariant &val);
    void onAxisChanged(QtProperty *property, const QVariant &val);
    void onCalibrationChanged(QtProperty *property, const QVariant &val);
    void onDynamicsChanged(QtProperty *property, const QVariant &val);
    void onLimitsChanged(QtProperty *property, const QVariant &val);
    void onMimicChanged(QtProperty *property, const QVariant &val);
    void onSafetyChanged(QtProperty *property, const QVariant &val);

  signals:
    void jointNameChanged(JointProperty *property, const QVariant &val);
    void parentLinkChanged(JointProperty *property, const QVariant &val);
    void originChanged(JointProperty *property);
    void axisChanged(JointProperty *property);
    void calibrationChanged(JointProperty *property);
    void dynamicsChanged(JointProperty *property);
    void limitsChanged(JointProperty *property);
    void mimicChanged(JointProperty *property);
    void safetyChanged(JointProperty *property);
    void valueChanged(JointProperty *property);

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
    boost::shared_ptr<URDFTransformer> tf_;
  };
}

#endif // __JOINT_PROPERTY_H__
