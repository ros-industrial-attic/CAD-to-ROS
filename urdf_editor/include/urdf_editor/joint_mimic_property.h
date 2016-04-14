#ifndef __JOINT_MIMIC_PROPERTY_H__
#define __JOINT_MIMIC_PROPERTY_H__

#include <QtCore>

#include <qttreepropertybrowser.h>
#include <qtvariantproperty.h>

#include <urdf_model/joint.h>


namespace urdf_editor
{
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
}

#endif // __JOINT_MIMIC_PROPERTY_H__
