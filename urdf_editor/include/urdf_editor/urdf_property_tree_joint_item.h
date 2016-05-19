#ifndef __URDF_PROPERTY_TREE_JOINT_ITEM_H__
#define __URDF_PROPERTY_TREE_JOINT_ITEM_H__

#include <QTreeWidgetItem>
#include <urdf_editor/urdf_types.h>
#include <urdf_editor/property_types.h>
#include <urdf_editor/joint_property.h>

namespace urdf_editor
{

  class URDFPropertyTreeJointItem: public QObject, public QTreeWidgetItem
  {
    Q_OBJECT
  public:
    explicit URDFPropertyTreeJointItem(urdf::JointSharedPtr joint, QStringList &link_names, QStringList &joint_names);
    virtual ~URDFPropertyTreeJointItem() {}

    QTreeWidgetItem *parent() const;

    QString getName();
    QString getParentLinkName();
    QString getChildLinkName();
    void setParentLinkName(QString name);
    void setChildLinkName(QString name);
    urdf::JointSharedPtr getData();
    JointPropertySharedPtr getProperty(); // this method should be removed once the property menu is moved to their respected propery class.
    void loadProperty(boost::shared_ptr<QtTreePropertyBrowser> property_editor);

  private slots:
    void on_jointNameChanged(JointProperty *property, const QVariant &val);
    void on_jointParentLinkChanged(JointProperty *property, const QVariant &val);

  signals:
    void jointNameChanged(JointProperty *property, QString current_name, QString new_name);
    void parentLinkChanged(JointProperty *property, QString current_name, QString new_name);
    void originChanged(JointProperty *property);
    void axisChanged(JointProperty *property);
    void calibrationChanged(JointProperty *property);
    void dynamicsChanged(JointProperty *property);
    void limitsChanged(JointProperty *property);
    void mimicChanged(JointProperty *property);
    void safetyChanged(JointProperty *property);
    void valueChanged(JointProperty *property);

  private:
    void updateDisplayText();

    QString name_;
    QString parent_link_name_;
    QString child_link_name_;
    urdf::JointSharedPtr joint_;
    QStringList &link_names_;
    QStringList &joint_names_;
    JointPropertySharedPtr property_;

  };
}

#endif // __URDF_PROPERTY_TREE_JOINT_ITEM_H__
