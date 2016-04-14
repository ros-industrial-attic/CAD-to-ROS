#ifndef URDF_PROPERTY_H
#define URDF_PROPERTY_H

#include <QtCore>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QMenu>
#include <urdf_parser/urdf_parser.h>
#include <urdf_editor/joint_property.h>
#include <urdf_editor/link_property.h>
#include <qttreepropertybrowser.h>
#include <urdf_editor/my_rviz.h>

namespace urdf
{
  class Joint;
  class Link;
}

namespace urdf_editor
{
  // forward declared
  class LinkProperty;
  class JointProperty;


  class URDFProperty : public QObject
  {
    Q_OBJECT
  public:
    URDFProperty(QTreeWidget *tree_widget, QWidget *browser_parent, QWidget *rviz_parent);
    ~URDFProperty();

    bool loadURDF(QString file_path);

    bool saveURDF(QString file_path);

    bool clear();

  private slots:
    void on_treeWidget_customContextMenuRequested(const QPoint &pos);

    void on_treeWidget_itemClicked(QTreeWidgetItem *item, int column);

    void on_propertyWidget_customContextMenuRequested(const QPoint &pos);

    void on_propertyWidget_linkNameChanged(LinkProperty *property, const QVariant &val);

    void on_propertyWidget_jointNameChanged(JointProperty *property, const QVariant &val);

    void on_propertyWidget_valueChanged();

  private:
    bool populateTreeWidget();

    void addToTreeWidget(boost::shared_ptr<urdf::Link> link, QTreeWidgetItem* parent);
    void addToTreeWidget(boost::shared_ptr<urdf::Joint> joint, QTreeWidgetItem* parent);

    void addModelLink(QTreeWidgetItem* parent);
    QTreeWidgetItem* addLinkTreeItem(QTreeWidgetItem* parent, boost::shared_ptr<urdf::Link> link);
    LinkPropertyPtr addLinkProperty(QTreeWidgetItem* item, boost::shared_ptr<urdf::Link> link);

    void addModelJoint(QTreeWidgetItem *parent);
    QTreeWidgetItem* addJointTreeItem(QTreeWidgetItem* parent, boost::shared_ptr<urdf::Joint> joint);
    JointPropertyPtr addJointProperty(QTreeWidgetItem *item, boost::shared_ptr<urdf::Joint> joint);

    QString getValidName(QString prefix, QList<QString> &current_names);

    bool isJoint(QTreeWidgetItem *item);
    bool isLink(QTreeWidgetItem *item);

    boost::shared_ptr<QtTreePropertyBrowser> property_editor_;
    boost::shared_ptr<urdf::ModelInterface> model_;
    QMap<boost::shared_ptr<urdf::Link>, QTreeWidgetItem *> joint_child_to_ctree_;

    // these map chain-tree-items to joint properties
    QMap<QTreeWidgetItem *, JointPropertyPtr> ctree_to_joint_property_;
    // and joint properties to chain-tree-items
    QMap<JointProperty *, QTreeWidgetItem *> joint_property_to_ctree_;

    // these map link-tree-items to link properties
    QMap<QTreeWidgetItem *, LinkPropertyPtr> ltree_to_link_property_;
    // and link properties to link-tree-items
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
