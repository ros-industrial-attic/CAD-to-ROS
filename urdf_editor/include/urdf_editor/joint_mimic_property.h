#ifndef __JOINT_MIMIC_PROPERTY_H__
#define __JOINT_MIMIC_PROPERTY_H__

#include <QtCore>

#include <urdf_editor/qt_types.h>
#include <urdf_editor/urdf_types_ext.h>


namespace urdf_editor
{
  class JointMimicProperty : public QObject
  {
    Q_OBJECT
  public:
    JointMimicProperty(urdf::JointMimicSharedPtr mimic, QStringList &joint_names);
    ~JointMimicProperty();

    void loadFactoryForManager(QtTreePropertyBrowserSharedPtr& property_editor);

    void loadData();

    QtProperty *getTopItem() { return top_item_; }

  private slots:
    void onValueChanged(QtProperty *property, const QVariant &val);

  signals:
    void valueChanged(QtProperty *property, const QVariant &val);

  private:
    urdf::JointMimicSharedPtr mimic_;
    QtVariantPropertyManager *manager_;
    QtVariantEditorFactory *factory_;
    QtProperty *top_item_;
    bool loading_;

    QStringList &joint_names_;
  };
}

#endif // __JOINT_MIMIC_PROPERTY_H__
