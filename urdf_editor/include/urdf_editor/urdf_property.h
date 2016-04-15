#ifndef URDF_PROPERTY_H
#define URDF_PROPERTY_H
#include <QtCore>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QMenu>
#include <urdf_parser/urdf_parser.h>
#include "urdf_editor/joint_property.h"
#include "urdf_editor/link_property.h"
#include <qttreepropertybrowser.h>
#include "urdf_editor/my_rviz.h"

namespace urdf_editor
{
  class URDFProperty : public QObject
  {
    Q_OBJECT
  public:
    URDFProperty(QTreeWidget *tree_widget, QWidget *browser_parent, QWidget *rviz_parent);
    ~URDFProperty();

    bool loadURDF(QString file_path);

    bool saveURDF(QString file_path);
    
    urdf_editor::MyRviz* getRvizWidget();

    bool clear();

  private slots:
    void on_treeWidget_customContextMenuRequested(const QPoint &pos);

    void on_treeWidget_itemClicked(QTreeWidgetItem *item, int column);

    void on_propertyWidget_customContextMenuRequested(const QPoint &pos);

    void on_propertyWidget_linkNameChanged(LinkProperty *property, const QVariant &val);

    void on_propertyWidget_jointNameChanged(JointProperty *property, const QVariant &val);

    void on_propertyWidget_valueChanged();

  private:
    bool buildTree();

    void addLink();
    void addLinkProperty(boost::shared_ptr<urdf::Link> link);

    void addJoint(QTreeWidgetItem *parent);
    void addJointProperty(QTreeWidgetItem *parent, boost::shared_ptr<urdf::Joint> joint);

    void setRvizPropertyForLinkProperty(LinkPropertyPtr link_property, QString link_name);
    
    QString getValidName(QString prefix, QList<QString> &current_names);
    bool isJoint(QTreeWidgetItem *item);

    boost::shared_ptr<QtTreePropertyBrowser> property_editor_;
    boost::shared_ptr<urdf::ModelInterface> model_;
    QMap<boost::shared_ptr<urdf::Link>, QTreeWidgetItem *> joint_child_to_ctree_;
    QMap<QTreeWidgetItem *, JointPropertyPtr> ctree_to_joint_property_;
    QMap<JointProperty *, QTreeWidgetItem *> joint_property_to_ctree_;
    QMap<QTreeWidgetItem *, LinkPropertyPtr> ltree_to_link_property_;
    QMap<LinkProperty *, QTreeWidgetItem *> link_property_to_ltree_;

    QStringList link_names_, joint_names_;
    QTreeWidgetItem *root_;
    QTreeWidgetItem *link_root_;
    QTreeWidgetItem *joint_root_;
    QTreeWidget *tree_widget_;
    QWidget *browser_parent_;
    urdf_editor::MyRviz *rviz_widget_;

  };
  typedef boost::shared_ptr<URDFProperty> URDFPropertyPtr;
}
#endif // URDF_TREE_PROPERTY_H
