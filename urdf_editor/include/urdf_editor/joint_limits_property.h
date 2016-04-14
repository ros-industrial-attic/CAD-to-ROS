#ifndef __JOINT_LIMITS_PROPERTY_H__
#define __JOINT_LIMITS_PROPERTY_H__

#include <QtCore>

#include <qttreepropertybrowser.h>
#include <qtvariantproperty.h>

#include <urdf_model/joint.h>


namespace urdf_editor
{
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
}

#endif // __JOINT_LIMITS_PROPERTY_H__