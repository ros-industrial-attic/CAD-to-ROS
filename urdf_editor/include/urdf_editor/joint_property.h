#ifndef JOINT_PROPERTY
#define JOINT_PROPERTY
#include <QtCore>
#include <boost/shared_container_iterator.hpp>
#include <qttreepropertybrowser.h>
#include <qtvariantproperty.h>
#include "urdf_editor/common.h"
#include <urdf_model/joint.h>
#include <urdf_editor/urdf_transforms.h>

namespace urdf_editor
{
  class OriginProperty : public QObject
  {
    Q_OBJECT
  public:
    OriginProperty(urdf::Pose &origin);
    ~OriginProperty();

    void loadFactoryForManager(boost::shared_ptr<QtTreePropertyBrowser> &property_editor);

    void loadData();

    QtProperty *getTopItem() { return top_item_; }

  private slots:
    void onValueChanged(QtProperty *property, const QVariant &val);

  signals:
    void valueChanged(QtProperty *property, const QVariant &val);

  private:
    urdf::Pose &origin_;
    QtVariantPropertyManager *manager_;
    QtVariantEditorFactory *factory_;
    QtProperty *top_item_;
    bool loading_;


  };

  class JointAxisProperty : public QObject
  {
    Q_OBJECT
  public:
    JointAxisProperty(urdf::Vector3 &axis);
    ~JointAxisProperty();

    void loadFactoryForManager(boost::shared_ptr<QtTreePropertyBrowser> &property_editor);

    void loadData();

    QtProperty *getTopItem() { return top_item_; }

  private slots:
    void onValueChanged(QtProperty *property, const QVariant &val);

  signals:
    void valueChanged(QtProperty *property, const QVariant &val);

  private:
    urdf::Vector3 &axis_;
    QtVariantPropertyManager *manager_;
    QtVariantEditorFactory *factory_;
    QtProperty *top_item_;
    bool loading_;

  };

  class JointSafetyProperty : public QObject
  {
    Q_OBJECT
  public:
    JointSafetyProperty(boost::shared_ptr<urdf::JointSafety> safety);
    ~JointSafetyProperty();

    void loadFactoryForManager(boost::shared_ptr<QtTreePropertyBrowser> &property_editor);

    void loadData();

    QtProperty *getTopItem() { return top_item_; }

  private slots:
    void onValueChanged(QtProperty *property, const QVariant &val);

  signals:
    void valueChanged(QtProperty *property, const QVariant &val);

  private:
    boost::shared_ptr<urdf::JointSafety> safety_;
    QtVariantPropertyManager *manager_;
    QtVariantEditorFactory *factory_;
    QtProperty *top_item_;
    bool loading_;

  };

  class JointMimicProperty : public QObject
  {
    Q_OBJECT
  public:
    JointMimicProperty(boost::shared_ptr<urdf::JointMimic> mimic, QStringList &joint_names);
    ~JointMimicProperty();

    void loadFactoryForManager(boost::shared_ptr<QtTreePropertyBrowser> &property_editor);

    void loadData();

    QtProperty *getTopItem() { return top_item_; }

  private slots:
    void onValueChanged(QtProperty *property, const QVariant &val);

  signals:
    void valueChanged(QtProperty *property, const QVariant &val);

  private:
    boost::shared_ptr<urdf::JointMimic> mimic_;
    QtVariantPropertyManager *manager_;
    QtVariantEditorFactory *factory_;
    QtProperty *top_item_;
    bool loading_;
    QStringList &joint_names_;

  };

  class JointCalibrationProperty : public QObject
  {
    Q_OBJECT
  public:
    JointCalibrationProperty(boost::shared_ptr<urdf::JointCalibration> calibration);
    ~JointCalibrationProperty();

    void loadFactoryForManager(boost::shared_ptr<QtTreePropertyBrowser> &property_editor);

    void loadData();

    QtProperty *getTopItem() { return top_item_; }

  private slots:
    void onValueChanged(QtProperty *property, const QVariant &val);

  signals:
    void valueChanged(QtProperty *property, const QVariant &val);

  private:
    boost::shared_ptr<urdf::JointCalibration> calibration_;
    QtVariantPropertyManager *manager_;
    QtVariantEditorFactory *factory_;
    QtProperty *top_item_;
    bool loading_;

  };

  class JointDynamicsProperty : public QObject
  {
    Q_OBJECT
  public:
    JointDynamicsProperty(boost::shared_ptr<urdf::JointDynamics> dynamics);
    ~JointDynamicsProperty();

    void loadFactoryForManager(boost::shared_ptr<QtTreePropertyBrowser> &property_editor);

    void loadData();

    QtProperty *getTopItem() { return top_item_; }

  private slots:
    void onValueChanged(QtProperty *property, const QVariant &val);

  signals:
    void valueChanged(QtProperty *property, const QVariant &val);

  private:
    boost::shared_ptr<urdf::JointDynamics> dynamics_;
    QtVariantPropertyManager *manager_;
    QtVariantEditorFactory *factory_;
    QtProperty *top_item_;
    bool loading_;

  };

  class JointLimitsProperty : public QObject
  {
    Q_OBJECT
  public:
    JointLimitsProperty(boost::shared_ptr<urdf::JointLimits> limits);
    ~JointLimitsProperty();

    void loadFactoryForManager(boost::shared_ptr<QtTreePropertyBrowser> &property_editor);

    void loadData();

    QtProperty *getTopItem() { return top_item_; }

  private slots:
    void onValueChanged(QtProperty *property, const QVariant &val);

  signals:
    void valueChanged(QtProperty *property, const QVariant &val);

  private:
    boost::shared_ptr<urdf::JointLimits> limits_;
    QtVariantPropertyManager *manager_;
    QtVariantEditorFactory *factory_;
    QtProperty *top_item_;
    bool loading_;

  };

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

    URDFTransformer tf_transformer_;

  };
  typedef boost::shared_ptr<JointProperty> JointPropertyPtr;
}
#endif // JOINT_PROPERTY

