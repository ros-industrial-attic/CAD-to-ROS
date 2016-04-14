#ifndef __JOINT_AXIS_PROPERTY_H__
#define __JOINT_AXIS_PROPERTY_H__

#include <QtCore>

#include <qttreepropertybrowser.h>
#include <qtvariantproperty.h>

#include <urdf_model/joint.h>


namespace urdf_editor
{
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
}

#endif // __JOINT_AXIS_PROPERTY_H__
