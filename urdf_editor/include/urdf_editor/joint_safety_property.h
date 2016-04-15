#ifndef __JOINT_SAFETY_PROPERTY_H__
#define __JOINT_SAFETY_PROPERTY_H__

#include <QtCore>

#include <urdf_editor/qt_types.h>
#include <urdf_editor/urdf_types_ext.h>


namespace urdf_editor
{
  class JointSafetyProperty : public QObject
  {
    Q_OBJECT
  public:
    JointSafetyProperty(urdf::JointSafetySharedPtr safety);
    ~JointSafetyProperty();

    void loadFactoryForManager(QtTreePropertyBrowserSharedPtr& property_editor);

    void loadData();

    QtProperty *getTopItem() { return top_item_; }

  private slots:
    void onValueChanged(QtProperty *property, const QVariant &val);

  signals:
    void valueChanged(QtProperty *property, const QVariant &val);

  private:
    urdf::JointSafetySharedPtr safety_;
    QtVariantPropertyManager *manager_;
    QtVariantEditorFactory *factory_;
    QtProperty *top_item_;
    bool loading_;
  };
}

#endif // __JOINT_SAFETY_PROPERTY_H__
